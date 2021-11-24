/*
	This class handles the low-level interface with the hardware.
*/
#include <Quintic.h>
#include <yarp/dev/ControlBoardInterfaces.h>						// Communicates with motor controllers on the robot?
#include <yarp/dev/PolyDriver.h>							// Device driver class
#include <yarp/os/LogStream.h>							// yarp::Info() and the like
#include <yarp/os/PeriodicThread.h>							// Enables the run() function
#include <yarp/os/Property.h>								// Don't know exactly what this does
#include <yarp/sig/Vector.h>								// yarp::sig::Vector class

class MultiJointController : virtual public yarp::os::PeriodicThread
{
	public:
		MultiJointController() : yarp::os::PeriodicThread(0.01) {};		// Empty constructor
		
		void configure_drivers(const std::string &local_port_name,
					const std::string &remote_port_name,
					const std::string &_name,
					const int &number_of_joints);
		bool update_state();							// Get new joint information				
		void close();								// Stop any control loops and close the device drivers
		void move_to_position(yarp::sig::Vector &position);			// Move the joints to a desired configuration
		void move_at_speed(const yarp::sig::Vector &speed);			// Move the joints at the desired speed
		yarp::sig::Vector get_positions();					// Get the current joint positions

		
		/*******************************************************************/
		virtual bool threadInit();						// Executed when start() is called
			
		// N.B. run() MUST be declared here for it to compile	
		virtual void run();							// Executed after threadInit()
		
		virtual void threadRelease();						// Executed when stop() is called
		/*******************************************************************/
		
		int n;									// Number of joints
		yarp::sig::Vector q, qdot;						// Joint positions, velocities
		
   		// These interface with the hardware
    		yarp::dev::IControlLimits	*limits;				// Joint limits?
		yarp::dev::IControlMode	*mode;					// Sets the control mode of the motor
		yarp::dev::IEncoders		*encoder;				// Joint position values (in degrees)
		yarp::dev::IVelocityControl	*controller;				// Motor-level velocity controller
		yarp::dev::PolyDriver		driver;				// Device driver

	private:		
		std::string name;							// Unique identifier
		yarp::sig::Vector pos, vel, acc, control;				// Desired joint state & control
		float start_time, end_time, elapsed_time;				// Used for timing on the trajectory
		Quintic trajectory;							// Custom trajectory class
	
};											// Semicolon needed after class declaration


/******************** Move the joints at the given speed ********************/
void MultiJointController::move_at_speed(const yarp::sig::Vector &speed)
{
	if(speed.size() > this->n)
	{
		yError() << "Vector length is greater than the number of joints!";
		for(int i = 0; i < this->n; i++)	this->controller->velocityMove(i,0.0);
	}
	else
	{
		for(int i = 0; i < speed.size(); i++)	this->controller->velocityMove(i, speed[i]);
	}
}

/******************** Move the joints to a desired configuration ********************/
void MultiJointController::move_to_position(yarp::sig::Vector &position)
{

	if(isRunning()) stop();							// If the control thread is running, stop it now
	
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
		
		update_state();							// Get the current joint state
		for(int i = 0; i < this->n; i++)
		{
			dq = position[i] - this->q[i];				// Distance to target
			this->limits->getVelLimits(i, &vMin, &vMax);			// Get velocity limits for ith joint
			if(dq >= 0)	dt = (15*dq)/(8*vMax);				// Time to cover distance at max speed			
			else		dt = (15*dq)/(8*vMin);					
			if(dt > this->end_time) this->end_time = dt;			// Update based on maximum time
		}
		this->end_time *= 2.0;							// Make the time a bit longer
		
		this->trajectory = Quintic(this->q.subVector(0,6), position, 0, this->end_time);	// New trajectory
		
		start();								// Go to threadInit()
	}	
}

/******************** Run immediately after start() is called ********************/
bool MultiJointController::threadInit()
{
	this->start_time = yarp::os::Time::now();					// Set new start time for trajectory tracking
	return true;
	// Now go to the run() function
}

/******************** Executed immediately after threadInit() ********************/
void MultiJointController::run()
{	
	update_state();									// Get new joint state information
	
	this->elapsed_time = yarp::os::Time::now() - this->start_time;			// Current elapsed time since start of control loop

	this->trajectory.get_state(this->pos, this->vel, this->acc, elapsed_time);		// Get desired joint state
	
	this->control = this->vel + 2.0*(this->pos - this->q);				// Feedforward + feedback term
	
	for(int i = 0; i < this->n; i++) this->controller->velocityMove(i,this->control[i]); // Send commands to joints
}

/******************** Executed immediately after stop() ********************/
void MultiJointController::threadRelease()
{
	for(int i = 0; i < this->n; i++) this->controller->velocityMove(i, 0.0);		// Ensure joints don't move
}

/******************** Get new joint encoder information ********************/
bool MultiJointController::update_state()
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

/******************** Get the joint positions ********************/
yarp::sig::Vector MultiJointController::get_positions()
{
	update_state();
	return this->q;
}

/******************** Stop any control loops and close the device drivers ********************/
void MultiJointController::close()
{
	if(isRunning()) stop();
	this->driver.close();
}

/******************** Proper Constructor ********************/
void MultiJointController::configure_drivers(	const std::string &local_port_name,
						const std::string &remote_port_name,
						const std::string &_name,
						const int &number_of_joints)
{
	this->name = _name;
	this->n = number_of_joints;
	
	// NOTE: For the arm, there are 7 joints for the arm and an additional
	// 9 for the hand.
	if(this->n > 7) this->n = 7;	
	
	// Resize vectors
	this->q.resize(this->n);
	this->pos.resize(this->n);
	this->vel.resize(this->n);
	this->acc.resize(this->n);
	this->control.resize(this->n);
	
	// First configure the device driver
	yarp::os::Property options;
	options.put("device", "remote_controlboard");
	options.put("local", local_port_name);
	options.put("remote", remote_port_name);
	
	this->driver.open(options);
	if(!this->driver.isValid())
	{
		yError() << "Unable to configure the device driver for" << this->name + ".";
		//yInfo() << yarp::dev::Drivers::factory().toString().c_str();
	}
	else	yInfo() << "Successfully configured the device driver for" << this->name + ".";

	// Next, configure the joint controllers
	if(!this->driver.view(this->controller))
	{		
		yError() << "Unable to configure the controller for" << this->name + ".";
		this->driver.close();
	}
	else if(!this->driver.view(this->mode))
	{
		yError() << "Unable to configure the control mode for" << this->name + ".";
	}
	else
	{
		for(int i = 0; i < this->n; i++)
		{
			this->mode->setControlMode(i, VOCAB_CM_VELOCITY);				// Set the motor in velocity mode
			this->controller->setRefAcceleration(i, std::numeric_limits<double>::max()); // CHANGE THIS?
			this->controller->velocityMove(i, 0.0);					// Ensure initial velocity is zero
		}
		
		yInfo() << "Successfully configured velocity control of" << this->name + ".";
	}
	
	// Now try and obtain joint limits
	if(!this->driver.view(this->limits))	yError() << "Unable to obtain joint limits for" << this->name + ".";
	else					yInfo() << "Successfully obtained limits for" << this->name + ".";
	
	// Finally, configure the encoders
	if(!this->driver.view(this->encoder))
	{
		yError() << "Unable to configure the encoder for" << this->name + ".";
	}
	else
	{
		for(int i = 0; i < 5; i++)
		{
			if(this->encoder->getEncoders(this->q.data()))				// Try and get initial values
			{
				yInfo() << "Successfully configured the encoder for" << this->name + ".";
				break;									// This should break the loop
			}
			if(i == 5)
			{
				yError() << "Could not obtain encoder values for" << this->name << "in 5 attempts.";
			}
			yarp::os::Time::delay(0.02);							// wait a little bit and try again
		}
	}
}

/******************** Configure the device drivers ********************
void MultiJointController::configure_drivers(const std::string &local_port_name,
						const std::string &remote_port_name,
						const std::string &_name,
						const int &number_of_joints)
{	
	this->n = number_of_joints;
	
	std::string name_ref = _name + ".";
	
	// First configure the device driver
	yarp::os::Property options;
	options.put("device", "remote_controlboard");
	options.put("local", local_port_name);
	options.put("remote", remote_port_name);
	
	this->driver.open(options);
	if(!this->driver.isValid())
	{
		yError() << "Unable to configure device. Here is a list of known devices:";
		yInfo() << yarp::dev::Drivers::factory().toString().c_str();
	}
	else	yInfo() << "Successfully configured the device driver for" << name_ref;

	// Next, configure the joint controllers
	if(!this->driver.view(this->controller))
	{
		std::cout << "Here." << std::endl;
		
		yError() << "Unable to configure the controller for" << name_ref;
		this->driver.close();
	}
	else if(!this->driver.view(this->mode))
	{
		yError() << "Unable to configure the control mode for" << name_ref;
	}
	else
	{
		for(int i = 0; i < this->n; i++)
		{
			this->mode->setControlMode(i, VOCAB_CM_VELOCITY);				// Set the motor in velocity mode
			this->controller->setRefAcceleration(i, std::numeric_limits<double>::max()); // CHANGE THIS?
			this->controller->velocityMove(i, 0.0);					// Ensure initial velocity is zero
		}
		
		yInfo() << "Successfully configured velocity control of" << name_ref;
	}
	
	// Now try and obtain joint limits
	if(!this->driver.view(this->limits))	yError() << "Unable to obtain joint limits for" << name_ref;
	else					yInfo() << "Successfully obtained limits for" << name_ref;
	
	// Finally, configure the encoders
	if(!this->driver.view(this->encoder))
	{
		yError() << "Unable to configure the encoder for" << name_ref;
	}
	else
	{
		for(int i = 0; i < 5; i++)
		{
			if(this->encoder->getEncoders(this->q.data()))				// Try and get initial values
			{
				yInfo() << "Successfully configured the encoder for" << name_ref;
				break;									// This should break the loop
			}
			if(i == 5)
			{
				yError() << "Could not obtain encoder values for" << this->name << "in 5 attempts.";
			}
			yarp::os::Time::delay(0.02);							// wait a little bit and try again
		}
	}
}


/*
class MultiJointController : public yarp::os::PeriodicThread
{
	public:
		MultiJointController() : yarp::os::PeriodicThread(100) {};		// Default constructor
		
		void configure_drivers(const std::string &local_port_name,
					const std::string &remote_port_name,
					const std::string &_name,
					const int &number_of_joints);			// Configure the device drivers
		
		void close() {this->driver.close();}					// Closes the device drivers
		
		bool update_state();							// Get new joint encoder information
		
		void move_to_position(yarp::sig::Vector &position);			// Move the joints to a desired position
		
		void move_at_speed(const yarp::sig::Vector &speed);			// Move the joints at the given speed
		
		/******************** Thread-		// Ensure the target is within joint limits
		double qMin, qMax;							// Upper and lower joint limits
		bool flag = false;							// Used to inform the user
		for(int i = 0; i < this->n; i++)
		{
			this->limits->getLimits(i, &qMin, &qMax);			// Get the limits for ith joint
			if(position[i] > qMax)	position[i] = qMax - 0.01;		// Just under the joint limit
			if(position[i] < qMin) position[i] = qMin + 0.01;		// Just above the joint limit
		}
		
		// Compute optimal time scaling
		double vMin, vMax;
		float dt, dq;
		float end_time = 0.0;
		
		update_state();							// Get the current joint state
		for(int i = 0; i < 7; i++)
		{
			dq = position[i] - this->q[i];				// Distance to target
			this->limits->getVelLimits(i, &vMin, &vMax);			// Get velocity limits for ith joint
			if(dq >= 0)	dt = (15*dq)/(8*vMax);				// Time to cover distance at max speed			
			else		dt = (15*dq)/(8*vMin);					
			if(dt > end_time) end_time = dt;				// Update based on maximum time
		}
		end_time *= 2.0;							// Make the time a bit longerrelated functions ********************
		bool initThread()							// Executed when start() is called
		{
			return true;
		}
		void run()								// Executed immediately after initThread()
		{
			/*
			get_joint_state();						// Update the joint state
			trajectory.get_state(pos,vel,acc,time);			// Get the desired state at the current time
			control = vel + 2.0*(pos - q);				// Feedforward + feedback term
				
			for(int i = 0; i < this->n; i++)
			{
				this->controller->velocityMove(i, control[i]);
			}
				
			if(time > end_time) stop();
				
		}
		
		void releaseThread()							// Executed when stop() is called?
		{
			/*
			for(int i = 0; i < this->n; i++)
			{
				this->controller->velocityMove(i, 0.0);		// Ensure joint does not move
			}
			
		}				
		/******************************************************************
		
		std::string name = "default";						// Unique identifier
		int n = 1;								// Number of joints
		yarp::sig::Vector q, qdot;						// Joint positions & velocities
	
		// These interface with the hardware
    		yarp::dev::IControlLimits	*limits;				// Joint limits?
		yarp::dev::IControlMode	*mode;					// Sets the control mode of the motor
		yarp::dev::IEncoders		*encoder;				// Joint position values (in degrees)
		yarp::dev::IVelocityControl	*controller;				// Motor-level velocity controller
		yarp::dev::PolyDriver		driver;				// Device driver
	
};											// Semicolon needed after class declaration

/******************** Get new joint encoder information ********************
bool MultiJointController::update_state()
{
	// TO DO: Get joint velocities
	
	if(this->encoder->getEncoders(this->q.data()))
	{
		return true;
	}
	else
	{
		yError() << "ArmController::update_state() : Could not obtain encoder values for " << this->name;
		return false;
	}
}

/******************** Move the joints to a desired configuration ********************
void MultiJointController::move_to_position(yarp::sig::Vector &position)
{
	if(position.size() > this->n)
	{
		yError() << "Input vector has more elements than the number of joints!";
	}
	else
	{
		// Ensure the target is within joint limits
		double qMin, qMax;							// Upper and lower joint limits
		bool flag = false;							// Used to inform the user
		for(int i = 0; i < this->n; i++)
		{
			this->limits->getLimits(i, &qMin, &qMax);			// Get the limits for ith joint
			if(position[i] > qMax)	position[i] = qMax - 0.01;		// Just under the joint limit
			if(position[i] < qMin) position[i] = qMin + 0.01;		// Just above the joint limit
		}
		
		// Compute optimal time scaling
		double vMin, vMax;
		float dt, dq;
		float end_time = 0.0;
		
		update_state();							// Get the current joint state
		for(int i = 0; i < 7; i++)
		{
			dq = position[i] - this->q[i];				// Distance to target
			this->limits->getVelLimits(i, &vMin, &vMax);			// Get velocity limits for ith joint
			if(dq >= 0)	dt = (15*dq)/(8*vMax);				// Time to cover distance at max speed			
			else		dt = (15*dq)/(8*vMin);					
			if(dt > end_time) end_time = dt;				// Update based on maximum time
		}
		end_time *= 2.0;							// Make the time a bit longer
	}
}

/******************** Move the joints at the given speed ********************
void MultiJointController::move_at_speed(const yarp::sig::Vector &speed)
{
	if(speed.size() > this->n)
	{
		yError() << "Vector length is greater than the number of joints!";
		for(int i = 0; i < this->n; i++)	this->controller->velocityMove(i,0.0);
	}
	else
	{
		for(int i = 0; i < speed.size(); i++)	this->controller->velocityMove(i, speed[i]);
	}
}

/******************** Configure the device drivers ********************
void MultiJointController::configure_drivers(const std::string &local_port_name,
						const std::string &remote_port_name,
						const std::string &_name,
						const int &number_of_joints)
{
	std::cout << "Number of joints: " << number_of_joints << std::endl;
	this->n = number_of_joints;
	std::cout << "A" << std::endl;
	this->q.resize(number_of_joints);
	std::cout << "B" << std::endl;
	this->qdot.resize(number_of_joints);
	std::cout << "C" << std::endl;
	std::cout << _name << std::endl;
	this->name = _name;
	std::cout << "D" << std::endl;
	std::string name_ref = this->name + ".";					// For printing information
	
	// First configure the device driver
	yarp::os::Property options;
	options.put("device", "remote_controlboard");
	options.put("local", local_port_name);
	options.put("remote", remote_port_name);
	
	this->driver.open(options);
	if(!this->driver.isValid())
	{
		yError() << "Unable to configure device. Here is a list of known devices:";
		yInfo() << yarp::dev::Drivers::factory().toString().c_str();
	}
	else	yInfo() << "Successfully configured the device driver for" << name_ref;
	
	// Next, configure the joint controllers
	if(!this->driver.view(this->controller))
	{
		yError() << "Unable to configure the controller for" << name_ref;
		this->driver.close();
	}
	else if(!this->driver.view(this->mode))
	{
		yError() << "Unable to configure the control mode for" << name_ref;
	}
	else
	{
		for(int i = 0; i < this->n; i++)
		{
			this->mode->setControlMode(i, VOCAB_CM_VELOCITY);				// Set the motor in velocity mode
			this->controller->setRefAcceleration(i, std::numeric_limits<double>::max()); // CHANGE THIS?
			this->controller->velocityMove(i, 0.0);					// Ensure initial velocity is zero
		}
		
		yInfo() << "Successfully configured velocity control of" << name_ref;
	}
	
	// Now try and obtain joint limits
	if(!this->driver.view(this->limits))	yError() << "Unable to obtain joint limits for" << name_ref;
	else					yInfo() << "Successfully obtained limits for" << name_ref;
	
	// Finally, configure the encoders
	if(!this->driver.view(this->encoder))
	{
		yError() << "Unable to configure the encoder for" << name_ref;
	}
	else
	{
		for(int i = 0; i < 5; i++)
		{
			if(this->encoder->getEncoders(this->q.data()))				// Try and get initial values
			{
				yInfo() << "Successfully configured the encoder for" << name_ref;
				break;									// This should break the loop
			}
			if(i == 5)
			{
				yError() << "Could not obtain encoder values for" << this->name << "in 5 attempts.";
			}
			yarp::os::Time::delay(0.02);							// wait a little bit and try again
		}
	}
}
*/

