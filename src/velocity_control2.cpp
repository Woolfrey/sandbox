#include <iostream>
#include <math.h>						// pow(x,n) function
#include <yarp/dev/ControlBoardInterfaces.h>			// Communicates with motor controllers on the robot?
#include <yarp/dev/PolyDriver.h>				// Device driver class
#include <yarp/os/LogStream.h>				// yarp::Info() and the like
#include <yarp/os/Network.h>					// Non so?
#include <yarp/os/Property.h>					// Don't know exactly what this does
#include <yarp/os/RpcServer.h>				// Allows communication between ports
#include <yarp/sig/Vector.h>					// For signal processing in YARP

/********************************* Simple Trajectory Class ***********************************/
class Quintic
{
	public:
	
	// Constructor
	Quintic(const yarp::sig::Vector &start_point, const yarp::sig::Vector &end_point, const double &start_time, const double &end_time)
	{
		this->p1 = start_point;
		this->p2 = end_point;
		this->t1 = start_time;
		this->t2 = end_time;
		this->m = this->p1.size();
		
		double dt = this->t2 - this->t1;
		this->a =   6*pow(dt,-5);
		this->b = -15*pow(dt,-4);
		this->c =  10*pow(dt,-3);
	}
	
	// Get the current desired position, velocity vectors	
	void getState(yarp::sig::Vector &pos, yarp::sig::Vector &vel, const double &t)
	{
		// Update the interpolation scalars
		double dt;
		if(t < this->t1) 	dt = 0.0;						// Remain at the start
		else if(t < this->t2) 	dt = t - this->t1;					// Remain at the end
		else if(t >= t2) 	dt = this->t2 - this->t1;				// Somewhere inbetween
		
		this->s =    this->a*pow(dt,5) +   this->b*pow(dt,4) +   this->c*pow(dt,3);	// Position (normalised)
		this->sd = 5*this->a*pow(dt,4) + 4*this->b*pow(dt,3) + 3*this->c*pow(dt,2);	// Velocity
		
		// Compute the desired state along the trajectory
		for(int i = 0; i < this->m; i++)
		{
			pos[i] = (1 - this->s)*this->p1[i] + this->s*this->p2[i];		// Desired position
			vel[i] = this->sd*(this->p2[i] - this->p1[i]);			// Desired velocity
		}
	}
	
	double end()
	{
		return this->t2;
	}
	
	private:
	
	int m;											// Number of dimensions
	
	yarp::sig::Vector p1, p2;								// Start point and end point
	
	double t1, t2;										// Start time and end time
	
	double a, b, c;									// Polynomial coefficients
	
	double s, sd;										// Interpolation scalars
};

/*************************************** Arm Controller ***********************************************/
class ArmController
{
	public:
		// Constructor
		ArmController(const std::string &local, const std::string &remote, const std::string &name)
		{
			this->name = name;							// Assign the name
			
			configureDriver(local, remote);
			configureControl();
			configureEncoder();
		}
		
		// Close the device driver for this arm
		void close()
		{
			this->driver.close();
		}
		
		// Move to a default home position
		void home()
		{
			yarp::sig::Vector target;						// Create target position
			
			// Arm
			target.push_back( 00.0);
			target.push_back( 20.0);
			target.push_back( 00.0);
			target.push_back( 15.0);
			target.push_back( 03.0);
			target.push_back(-05.0);
			target.push_back( 02.0);
			
			// Hand
			target.push_back( 00.0);
			target.push_back( 20.0);
			target.push_back( 20.0);
			target.push_back( 20.0);
			
			// Fingers
			target.push_back( 10.0);
			target.push_back( 10.0);
			target.push_back( 10.0);
			target.push_back( 10.0);
			target.push_back( 10.0);
			
			moveToConfiguration(target,2.0);
		}
		
		void moveToConfiguration(const yarp::sig::Vector &target, const double &time)
		{
			this->encoder->getEncoders(this->position.data());			// Get current joint values
			Quintic trajectory(this->position, target, 0.0, time);		// Create trajectory object
			
			yarp::sig::Vector pos(this->n), vel(this->n);				// Vectors for desired state
			
			double t0, t;
			t0 = yarp::os::Time::now();						// Get starting time
			
			// Run through control loop
			do
			{	
				t = yarp::os::Time::now() - t0;				// Elapsed time
				trajectory.getState(pos, vel, t);				// Get desired position, velocity
				
				for(int i = 0; i < 7; i++)
				{
					this->controller->velocityMove(i,
					vel[i] + 2.0*(pos[i] - this->position[i]));		// P control with feedforward
				}
				
				this->encoder->getEncoders(this->position.data());		// Get new encoder values
				
				yarp::os::Time::delay(0.001);					

			}
			while(t < trajectory.end());
			
			for(int i = 0; i < 7; i++) this->controller->velocityMove(i,0); 	// Ensure joint stops moving
		}
	
	private:

		// Properties
		int n;										// Number of joints
		std::string name = "this device";						// A name to identify the object
		yarp::dev::IControlMode* mode;						// Sets the control mode of the motor
		yarp::dev::IEncoders* encoder;						// Reads encoder values
		yarp::dev::IVelocityControl* controller;					// Motor-level velocity controller
		yarp::dev::PolyDriver driver;							// Device driver
		yarp::sig::Vector position;							// Current joint positions
		
		// Functions
		bool configureControl()
		{
			if(!this->driver.view(this->controller))
			{
				yError() << "Unable to configure the controller for" << this->name;
				this->driver.close();
				return 0;
			}
			else if(!this->driver.view(this->mode))
			{
				yError() << "Unable to configure the control mode for" << this->name;
				return 0;
			}
			else
			{
				this->controller->getAxes(&this->n);
				
				for(int i = 0; i < this->n; i++)
				{
					this->mode->setControlMode(i, VOCAB_CM_VELOCITY);	// Set the motor in velocity mode
					this->controller->setRefAcceleration(i,50);		// CHANGE THIS?
					this->controller->velocityMove(i,0);			// Ensure initial velocity is zero
				}
				yInfo() << "Successfully configures velocity control of" << this->name;
				return 1;
			}
		}
						
		bool configureDriver(const std::string &local, const std::string &remote)
		{
			yarp::os::Property options;						// Object to temporarily store options
			options.put("device", "remote_controlboard");				
			options.put("local", local);						// Local port names
			options.put("remote", remote);					// Where we want to connect to
	
			this->driver.open(options);						// Assign the options to the driver
	
			if(!this->driver.isValid())						// Check if requested driver is available	
			{
				yError("Device not available. Here are the known devices:");
				std::printf("%s", yarp::dev::Drivers::factory().toString().c_str());	
				return 0;							// Shut down
			}			
			else
			{
				yInfo() << "Successfully configured device driver for" << this->name;
				return 1;
			}
		}
		
		bool configureEncoder()
		{
			if(!this->driver.view(this->encoder))
			{
				yError() << "Could not configure the encoder for" << this->name;
				return 0;
			}
			else
			{
				this->encoder->getAxes(&this->n);				// Get the number of axes
				this->position.resize(this->n);				// Resize the vector accordingly
				yInfo() << "Successfully configured the encoder for " << this->name;
				return 1;
			}
		}
};												// Need this after a class declaration

/***************************************** Main *****************************************************/

int main(int argc, char *argv[])
{
	yarp::os::Network yarp;								// Communicate over yarp network?
	
	// Configure the left arm
	std::string local = "/local/left";
	std::string remote = "/icubSim/left_arm";
	std::string name = "left_arm";
	ArmController left_arm(local, remote, name);
	
	// Configure the right arm
	local = "/local/right";
	remote = "/icubSim/right_arm";
	name = "right_arm";
	ArmController right_arm(local, remote, name);
	
	left_arm.home();
	right_arm.home();
	
	yarp::os::Time::delay(1);
	
	left_arm.close();
	right_arm.close();
	
	return 0;										// No problems
}
