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
	
	// Empty constructor
	Quintic()
	{
	}
	
	// Set new values
	void setTrajectory(const yarp::sig::Vector &start_point, const yarp::sig::Vector &end_point, const double &start_time, const double &end_time)
	{
		// Assign new values
		this->p1 = start_point;
		this->p2 = end_point;
		this->t1 = start_time;
		this->t2 = end_time;
		
		this->m = p1.size();								// Number of dimensions
		
		// Compute new polynomial coefficients
		double dt = end_time - start_time;
		this->a =   6*pow(dt,-5);
		this->b = -15*pow(dt,-4);
		this->c =  10*pow(dt,-3);
	}
	
	void getState(yarp::sig::Vector &pos, yarp::sig::Vector &vel, const double &t)
	{
		// Compute time along the trajectory
		double dt;
		if(t < this->t1) 	dt = 0.0;						// Remain at the start
		else if(t < this->t2) 	dt = t - this->t1;					// Remain at the end
		else if(t >= t2) 	dt = this->t2 - this->t1;				// Somewhere inbetween
		
		// Update the interpolation scalars
		this->s =    this->a*pow(dt,5) +   this->b*pow(dt,4) +   this->c*pow(dt,3);	// Position (normalised)
		this->sd = 5*this->a*pow(dt,4) + 4*this->b*pow(dt,3) + 3*this->c*pow(dt,2);	// Velocity
		
		// Compute the desired state along the trajectory
		for(int i = 0; i < this->m; i++)
		{
			pos[i] = (1 - this->s)*this->p1[i] + this->s*this->p2[i];		// Desired position
			vel[i] = this->sd*(this->p2[i] - this->p1[i]);			// Desired velocity
		}
	}
	
	// End time for the trajectory
	double end()
	{
		return this->t2;
	}
	
	
	private:
		int m;										// Number of dimensions
		
		double t1, t2;									// Start time and end time
		
		yarp::sig::Vector p1, p2;							// Start point and end point
		
		double a, b, c;								// Polynomial coefficients
		
		double s, sd;									// Interpolation coefficients
	
};

/*************************************** Arm Controller ***********************************************/
class ArmController: private Quintic
{
	public:
		// Constructor
		ArmController(const std::string &local, const std::string &remote, const std::string &name)
		{
			this->name = name;							// Assign the name
			configureDriver(local, remote);					// This must be called before the next 2
			configureControl();
			configureEncoder();
		}
		
		// Close the device driver for this arm
		void close()
		{
			this->driver.close();
		}
		
		// Send velocity commands to actively track trajectory
		bool control(const double &t)
		{
			yarp::sig::Vector pos(this->n), vel(this->n);				// BETTER WAY TO DO THIS?
			
			this->trajectory.getState(pos, vel, t);				// Get desired state along the trajectory
			
			this->encoder->getEncoders(this->position.data());			// Get current joint state	
			
			for(int i = 0; i < 7; i++) move(i, vel[i] + 5.0*(pos[i] - this->position[i]));
			
			if(t >= this->trajectory.end()) return 1;				// Reached the end of the trajectory
			else 				 return 0;				// Not yet reached the end		
		}
		
		// Send a single velocity command to a single joint
		void move(const int &joint, const double &speed)
		{
			this->controller->velocityMove(joint, speed);
		}
		
		// Set a target configuration to achieve at a given time
		void setTarget(const yarp::sig::Vector &target, const double &time)
		{
			this->encoder->getEncoders(this->position.data());			// Get current joint configuration
			this->trajectory.setTrajectory(this->position, target, 0, time);	
		}
		
		void stop()
		{
			for(int i = 0; i < 7; i++) move(i,0);
		}

	private:

		// Properties
		int n;										// Number of joints
		std::string name = "this device";						// A name to identify the object
    		yarp::dev::IControlLimits* limits;
		yarp::dev::IControlMode* mode;						// Sets the control mode of the motor
		yarp::dev::IEncoders* encoder;						// Reads encoder values
		yarp::dev::IVelocityControl* controller;					// Motor-level velocity controller
		yarp::dev::PolyDriver driver;							// Device driver
		yarp::sig::Vector position;							// Current joint positions
		
		Quintic trajectory;
		
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
				this->controller->getAxes(&this->n);				// NO. OF JOINTS IS SET HERE
				
				for(int i = 0; i < this->n; i++)
				{
					this->mode->setControlMode(i, VOCAB_CM_VELOCITY);	// Set the motor in velocity mode
					this->controller->setRefAcceleration(i,std::numeric_limits<double>::max()); // CHANGE THIS?
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
			bool success = true;
			
			if(!this->driver.view(this->encoder))
			{
				yError() << "Could not configure the encoder for" << this->name;
				success = false;
			}
			else
			{
				this->encoder->getAxes(&this->n);				// Get the number of axes
				this->position.resize(this->n);				// Resize the vector accordingly
				
				for(int i = 0; i < 5; i++)
				{
					if(this->encoder->getEncoders(this->position.data())) // Get initial encoder values
					{
						yInfo() << "Successfully configured the encoder for" << this->name;
						success = true;
					}
					if(i == 5)
					{
						yError() << "Could not obtain encoder values for" << this->name << "in 5 attempts.";
						success = false;
					}
					yarp::os::Time::delay(0.01);
				}
			}
			return success;
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
	ArmController left_arm(local, remote, name);						// Create and configure arm controller object
	
	// Configure the right arm
	local = "/local/right";
	remote = "/icubSim/right_arm";
	name = "right_arm";
	ArmController right_arm(local, remote, name);						// Create and configure arm controller object
	
	// Move to a starting position
	yarp::sig::Vector home(16);
	home[0] = 00.0;
	home[1] = 20.0;
	home[2] = 00.0;
	home[3] = 15.0;
	home[4] = 00.0;
	home[5] = 00.0;
	home[6] = 00.0;
	
	left_arm.setTarget(home,2.0);								// Assign new control target
	right_arm.setTarget(home,2.0);						
	double start = yarp::os::Time::now();
	double elapsed_time;
	bool complete = false;
	do
	{
		elapsed_time = yarp::os::Time::now() - start;					// Time since start of loop
		complete = left_arm.control(elapsed_time);					// 
		complete &= right_arm.control(elapsed_time);
		yarp::os::Time::delay(0.001);
		
	}while(!complete);
	
	left_arm.stop();
	right_arm.stop();

	// Configure communication across yarp
	yarp::os::RpcServer port;								// Create a port for sending and receiving information
	port.open("/command");									// Open the port with the name /command
	yarp::os::Bottle input;								// Store information from user input
	yarp::os::Bottle output;								// Store information to send to user
	std::string command;									// Response message, command from user
	
	// Run the control loop
	bool active = true;
	while(active)
	{
		output.clear();								// Clear any previous
		
		bool control = false;								// Don't move the robot
		
		port.read(input, true);							// Get any commands over the network
		command = input.toString();							// Convert to a string
		
		// Shut down the robot
		if(command == "close")
		{
			output.addString("Arrivederci");
			active = false;							// Shut down the while loop
			
			left_arm.setTarget(home, 2.0);
			right_arm.setTarget(home, 2.0);
			control = true;							// Activate the control loop
		}
		
		// Return to a home position
		else if(command == "home")
		{
			output.addString("Casa");
			left_arm.setTarget(home, 2.0);
			right_arm.setTarget(home, 2.0);
			control = true;							// Activate the control loop
		}
		
		// Extend out both arms
		else if(command == "receive")
		{
			output.addString("Grazie");					
			
			yarp::sig::Vector setpoint(7);
			setpoint[0] = -50.0;
			setpoint[1] =  00.0;
			setpoint[2] =  00.0;
			setpoint[3] =  50.0;
			setpoint[4] = -60.0;
			setpoint[5] =  00.0;
			setpoint[6] =  00.0;
			
			left_arm.setTarget(setpoint, 3.0);
			right_arm.setTarget(setpoint, 3.0);
			control = true;							// Activate the control loop
		}

		// Extend one arm to shake hands
		else if(command == "shake")
		{
			output.addString("Piacere");
			
			yarp::sig::Vector setpoint(7);
			setpoint[0] = -50.0;
			setpoint[1] =  20.0;
			setpoint[2] =  00.0;
			setpoint[3] =  50.0;
			setpoint[4] =  00.0;
			setpoint[5] =  00.0;
			setpoint[6] =  00.0;
			
			left_arm.setTarget(home, 2.0);
			right_arm.setTarget(setpoint, 2.0);
			control = true;							// Activate the control loop
		}

		// Wave one arm
		else if(command == "wave")
		{
			output.addString("Ciao");
			
			yarp::sig::Vector setpoint(7);
			setpoint[0] =  -30.0;
			setpoint[1] =   50.0;
			setpoint[2] =  -60.0;
			setpoint[3] =  105.0;
			setpoint[4] =   30.0;
			setpoint[5] =   00.0;
			setpoint[6] =   00.0;
			
			left_arm.setTarget(home, 2.0);
			right_arm.setTarget(setpoint, 2.0);
			control = true;
		}
		
		// Unknown command
		else
		{
			output.addString("Cosa");
		}
		
		port.reply(output);								// Send response to user
		
		if(control)									// Control active
		{
			
			double t0 = yarp::os::Time::now();
			double t;
			
			bool finished = false;
			
			while(!finished)
			{
				t = yarp::os::Time::now() - t0;				// Elapsed time in control loop
				finished = left_arm.control(t);				// Feedback control on left arm trajectory
				finished &= right_arm.control(t);				// Feedback control on right arm trajectory
				yarp::os::Time::delay(0.001);
			}
		}
		left_arm.stop();
		right_arm.stop();
		
		output.clear();
		output.addString("Action complete. Please input next command.");
		port.reply(output);
	}
	
	left_arm.close();
	right_arm.close();
	
	return 0;										// No problems
}

