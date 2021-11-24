#include <Quintic.h>									// Custom trajectory class
#include <yarp/dev/ControlBoardInterfaces.h>						// Communicates with motor controllers on the robot?
#include <yarp/dev/PolyDriver.h>							// Device driver class
#include <yarp/os/LogStream.h>							// yarp::Info() and the like
#include <yarp/os/Property.h>								// Don't know exactly what this does

class MultiJointController
{
	public:
		MultiJointController(){};						// Default constructor
		
		void configure_drivers(const std::string &local_port_name,		// Configure communication with robot
					const std::string &remote_port_name,
					const std::string &_name,
					const int &number_of_joints);
					
		bool get_encoder_values();						// Get new state information
		
		void close() { this->driver.close();}					// Close the drivers
		
		void set_joint_trajectory(yarp::sig::Vector &position);		// Set a new joint trajectory object
		
		void move_at_speed(const yarp::sig::Vector &speed);			// Send velocity commands to each individual joint
		
		void joint_control(const double &time);				// Solve control to track trajectory					
		
		yarp::sig::Vector q, qdot;
		
	private:
		// Variables used internally in this class
		int n;									// Number of joints
		std::string name;							// Unique identifier
		yarp::sig::Vector pos, vel, acc, ctrl;				// Joint state and control vectors

		Quintic trajectory;							// Joint trajectory object
		
		
	   	// These interface with the hardware
    		yarp::dev::IControlLimits*	limits;				// Joint limits?
		yarp::dev::IControlMode*	mode;					// Sets the control mode of the motor
		yarp::dev::IEncoders*		encoder;				// Joint position values (in degrees)
		yarp::dev::IVelocityControl*	controller;				// Motor-level velocity controller
		yarp::dev::PolyDriver		driver;				// Device driver
		
};											// Semicolon needed after class declaration

/******************** Get new joint encoder information ********************/
bool MultiJointController::get_encoder_values()
{
	// TO DO: Get joint velocities
	
	if(this->encoder->getEncoders(this->q.data()))
	{
		return true;
	}
	else
	{
		yError() << "ArmController::update_state() : Could not obtain encoder values for " << this->name + ".";
		return false;
	}
}

/******************** Set a new joint trajectory ********************/
void MultiJointController::set_joint_trajectory(yarp::sig::Vector &position)
{
	// Check the inputs are sound
	if(position.size() > this->n)
	{
		yError() << "MultiJointController::move_to_position() : Length of input vector is greater than the number of joints!";
	}
	else
	{
		// Ensure the target is within joint limits
		double qMin, qMax;							// Upper and lower joint limits
		for(int i = 0; i < this->n; i++)
		{
			this->limits->getLimits(i, &qMin, &qMax);			// Get the limits for ith joint
			if(position[i] > qMax)	position[i] = qMax - 0.01;		// Just under the joint limit
			if(position[i] < qMin) position[i] = qMin + 0.01;		// Just above the joint limit
		}
		
		// Compute optimal time scaling
		double vMin, vMax;							// Speed limits
		double dt, dq;								// Distance in time, space
		double endTime;							// To be computed here
		
		get_encoder_values();							// Get the current joint state
		for(int i = 0; i < this->n; i++)
		{
			dq = position[i] - this->q[i];				// Distance to target
			this->limits->getVelLimits(i, &vMin, &vMax);			// Get velocity limits for ith joint
			if(dq >= 0)	dt = (15*dq)/(8*vMax);				// Time to cover distance at max speed			
			else		dt = (15*dq)/(8*vMin);					
			if(dt > endTime) endTime = dt;				// Update based on maximum time
		}
		endTime *= 1.5;							// Make the time a bit longer
		
		this->trajectory = Quintic(this->q, position, 0, endTime);		// New trajectory
	}	
}

/******************** Solve the trajectory tracking control ********************/
void MultiJointController::joint_control(const double &time)
{
	this->trajectory.get_state(this->pos, this->vel, this->acc, time);		// Get the desired state from the trajectory object
	
	this->ctrl = vel + 2.0*(this->pos - this->q);					// Feedforward + feedback control
	
	move_at_speed(this->ctrl);							// Send commands to joints
}

/******************** Send velocity commands to each individual joint ********************/
void MultiJointController::move_at_speed(const yarp::sig::Vector &speed)
{
	if(speed.size() != this->n)
	{
		yError("MultiJointController::move_at_speed() : Length of input vector does not match the number of joints!");
	}
	else
	{
		// CHECK THE SPEED LIMITS HERE?
		for(int i = 0; i < this->n; i++) this->controller->velocityMove(i,speed[i]);
	}
}

/******************** As it says on the label ********************/
void MultiJointController::configure_drivers(	const std::string &local_port_name,
						const std::string &remote_port_name,
						const std::string &_name,
						const int &number_of_joints)
{
	this->name = _name;
	this->n = number_of_joints;
	
	// Resize vectors
	this->q.resize(this->n);							// Actual joint position vector
	this->qdot.resize(this->n);							// Actual joint velocity vector
	this->pos.resize(this->n);							// Desired joint position vector
	this->vel.resize(this->n);							// Desired joint velocity vector
	this->acc.resize(this->n);							// Desired joint acceleration vector
	this->ctrl.resize(this->n);							// Joint velocity control vector
	
	// First configure the device driver
	yarp::os::Property options;
	options.put("device", "remote_controlboard");
	options.put("local", local_port_name);
	options.put("remote", remote_port_name);
	
	bool totalSuccess = true;
	
	this->driver.open(options);
	if(!this->driver.isValid())
	{
		yError() << "Unable to configure the device driver for" << this->name + ".";
		//yInfo() << yarp::dev::Drivers::factory().toString().c_str();
		totalSuccess == false;
	}
	//else	yInfo() << "Successfully configured the device driver for" << this->name + ".";

	// Next, configure the joint controllers
	if(!this->driver.view(this->controller))
	{		
		yError() << "Unable to configure the controller for" << this->name + ".";
		this->driver.close();
		totalSuccess = false;
	}
	else if(!this->driver.view(this->mode))
	{
		yError() << "Unable to configure the control mode for" << this->name + ".";
		totalSuccess= false;
	}
	else
	{
		for(int i = 0; i < this->n; i++)
		{
			this->mode->setControlMode(i, VOCAB_CM_VELOCITY);				// Set the motor in velocity mode
			this->controller->setRefAcceleration(i, std::numeric_limits<double>::max()); // CHANGE THIS?
			this->controller->velocityMove(i, 0.0);					// Ensure initial velocity is zero
		}
		
		//yInfo() << "Successfully configured velocity control of" << this->name + ".";
	}
	
	// Now try and obtain joint limits
	if(!this->driver.view(this->limits))
	{
		yError() << "Unable to obtain joint limits for" << this->name + ".";
		totalSuccess = false;
	}
	//else					yInfo() << "Successfully obtained limits for" << this->name + ".";
	
	// Finally, configure the encoders
	if(!this->driver.view(this->encoder))
	{
		yError() << "Unable to configure the encoder for" << this->name + ".";
		totalSuccess = false;
	}
	else
	{
		for(int i = 0; i < 5; i++)
		{
			if(this->encoder->getEncoders(this->q.data()))				// Try and get initial values
			{
				//yInfo() << "Successfully configured the encoder for" << this->name + ".";
				break;									// This should break the loop
			}
			if(i == 5)
			{
				yError() << "Could not obtain encoder values for" << this->name << "in 5 attempts.";
				totalSuccess = false;
			}
			yarp::os::Time::delay(0.02);							// wait a little bit and try again
		}
	}
	if(totalSuccess) yInfo() << "Successfully configured drivers for the" << this->name + ".";
}
