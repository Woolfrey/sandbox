#include <Quintic.h>								// Custom trajectory class
#include <yarp/dev/ControlBoardInterfaces.h>					// Communicates with motor controllers on the robot?
#include <yarp/dev/PolyDriver.h>						// Device driver class
#include <yarp/os/LogStream.h>							// yarp::Info() and the like
#include <yarp/os/Property.h>							// Don't know exactly what this does

class MultiJointController
{
	public:
		MultiJointController() {};					// Empty constructer
		
		void configure_drivers(const std::string &local_port_name,	// Configure communication with the robot
					const std::string &remote_port_name,
					const std::string &_name,
					const int &number_of_joints);
					
		void close() { this->driver.close();}				// Close the drivers
		
		yarp::sig::Vector get_joint_positions() {return this->q;}	// As it says on the label
		yarp::sig::Vector get_joint_velocities() {return this->qdot;}	// Anche
		
		bool read_encoders();						// Read encoder values
		
		void set_joint_trajectory(yarp::sig::Vector &target);		// Set a new target for the joints
		
		void joint_control(const double &time);				// Solve feedback control for given time
		
		void move_at_speed(const yarp::sig::Vector speed);		// Send velocity commands to the motors
		
		double get_joint_weight(const int &j);
		
	private:
		int n;								// Number of joints
		
		Quintic trajectory;						// Joint-level trajectory generator
		
		std::string name;						// Identifies this object
		
		yarp::sig::Vector q, qdot, qMin, qMax;				// Position, velocity, joint limits
		yarp::sig::Vector pos, vel, acc;				// Desired state for control purposes
		
	   	// These interface with the hardware
    		yarp::dev::IControlLimits*	limits;				// Joint limits?
		yarp::dev::IControlMode*	mode;				// Sets the control mode of the motor
		yarp::dev::IEncoders*		encoder;			// Joint position values (in degrees)
		yarp::dev::IVelocityControl*	controller;			// Motor-level velocity controller
		yarp::dev::PolyDriver		driver;				// Device driver
		
};										// Semicolon needed after a class declaration

/******************** As it says on the label ********************/
void MultiJointController::configure_drivers(const std::string &local_port_name,
						const std::string &remote_port_name,
						const std::string &_name,
						const int &number_of_joints)
{
	this->name = _name;
	this->n = number_of_joints;
	
	// Resize vectors
	this->q.resize(this->n);							// Actual joint position vector
	this->qdot.resize(this->n);							// Actual joint velocity vector
	this->qMin.resize(this->n);							// Lower joint limits
	this->qMax.resize(this->n);							// Upper joint limits
	this->pos.resize(this->n);							// Desired joint position vector
	this->vel.resize(this->n);							// Desired joint velocity vector
	this->acc.resize(this->n);							// Desired joint acceleration vector
	
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
		totalSuccess == false;
	}

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
			this->controller->setRefAcceleration(i, std::numeric_limits<double>::max()); 	// CHANGE THIS?
			this->controller->velocityMove(i, 0.0);						// Ensure initial velocity is zero
		}
	}
	
	// Now try and obtain joint limits
	if(!this->driver.view(this->limits))
	{
		yError() << "Unable to obtain joint limits for" << this->name + ".";
		totalSuccess = false;
	}
	else
	{
		// Save them internally in radians for later use
		for(int i = 0; i < this->n; i++)
		{
			this->limits->getLimits(i, &this->qMin[i], &this->qMax[i]);
		}
		
//		yInfo() << "Here are the joint limits for the " << this->name << " in deg:";
//		std::cout << this->qMin.toString() << std::endl;
		
		this->qMin *= M_PI/180;
		this->qMax *= M_PI/180;
		
//		yInfo() << "And here they are in rad:";
//		std::cout << this->qMin.toString() << std::endl;
	}
	
	// Finally, configure the encoders
	if(!this->driver.view(this->encoder))
	{
		yError() << "Unable to configure the encoder for" << this->name + ".";
		totalSuccess = false;
	}
	else
	{
		yarp::sig::Vector temp(this->n);						// Temporary storage location
		
		for(int i = 0; i < 5; i++)
		{
			if(this->encoder->getEncoders(temp.data()))				// Try and get initial values
			{
				break;								// This should break the loop
			}
			if(i == 5)
			{
				yError() << "Could not obtain encoder values for" << this->name << "in 5 attempts.";
				totalSuccess = false;
			}
			yarp::os::Time::delay(0.05);						// wait a little bit and try again
		}
		
		this->q = temp*M_PI/180;							// Make sure the values are in radians
	}
	if(totalSuccess) yInfo() << "Successfully configured drivers for the" << this->name + ".";
}

/******************** Get new joint encoder information ********************/
bool MultiJointController::read_encoders()
{
	bool success = true;
	
	for(int i = 0; i < this->n; i++)
	{
		success &= this->encoder->getEncoder(i, &this->q[i]);
		success &= this->encoder->getEncoderSpeed(i, &this->qdot[i]);
	}
	
//	yInfo() << "Here are the joint angles for " << this->name << " in degrees:";
//	std::cout << this->q.toString() << std::endl;
	
	this->q *= M_PI/180;
	this->qdot *= M_PI/180;
	
//	yInfo() << " And here they are in radians:";
//	std::cout << this->q.toString() << std::endl;

	if(!success) yError() << "ArmController::update_state() : Could not obtain encoder values for " << this->name + ".";
	
	return success;
}

/******************** Set a new joint trajectory ********************/
void MultiJointController::set_joint_trajectory(yarp::sig::Vector &target)
{
	// Check the inputs are sound
	if(target.size() > this->n)
	{
		yError() << "MultiJointController::set_joint_trajectory() : Length of input vector is greater than the number of joints!";
	}
	else
	{
		// Ensure the target is within joint limits
		for(int i = 0; i < this->n; i++)
		{
			if(target[i] > this->qMax[i])	target[i] = this->qMax[i] - 0.01;
			if(target[i] < this->qMin[i])	target[i] = this->qMin[i] + 0.01;
		}
		
		// Compute optimal time scaling
		double vMin, vMax, dt, dq;
		double endTime = 2.0;
		
		for(int i = 0; i < this->n; i++)
		{
			dq = abs(target[i] - this->q[i]);			// Distance to target
			this->limits->getVelLimits(i, &vMin, &vMax);		// Get velocity limits for ith joint
			vMax *= M_PI/180;
			//std::cout << "vMin: " << vMin << " vMax: " << vMax << std::endl;
			if(dq >= 0)	dt = (15*dq)/(8*vMax);			// Optimal time scaling at max velocity
			
			if(dt > endTime) endTime = dt;				// Overwrite based on largest observed time
		}
		
		//yInfo() << "Total trajectory time: " << endTime;
		
		this->trajectory = Quintic(this->q, target, 0.0, endTime);
	}
}

/******************** Solve the trajectory tracking control ********************/
void MultiJointController::joint_control(const double &time)
{
	this->trajectory.get_state(this->pos, this->vel, this->acc, time);		// Get the desired state from the trajectory object
	
//	yInfo() << "Here is the desired position for time " << time <<":";
//	std::cout << this->pos.toString() << std::endl;

	move_at_speed(this->vel + 5.0*(this->pos - this->q));				// Send commands to the motors
}

/******************** Send velocity commands to each individual joint ********************/
void MultiJointController::move_at_speed(const yarp::sig::Vector speed)
{
	if(speed.size() != this->n)
	{
		yError("MultiJointController::move_at_speed() : Length of input vector does not match the number of joints!");
	}
	else
	{
		// CHECK THE SPEED LIMITS HERE?
		for(int i = 0; i < this->n; i++) this->controller->velocityMove(i,speed[i]*180/M_PI);
	}
}

/******************** Get the penalty term for avoiding joint limits ********************/
double MultiJointController::get_joint_weight(const int &j)
{
	double qMin, qMax;
	this->limits->getLimits(j, &qMin, &qMax);					// Get the lower and upper limits for this joint
	
	double range = qMax - qMin;							// Distance between joints
	
	double upper = qMax - this->q[j];						// Distance to upper limit
	double lower = this->q[j] - qMin;						// Distance from lower limit
	
	double dpdq = pow(upper,-2) - pow(lower, -2);					// Partial derivative of the penalty function
	
	if(dpdq*this->qdot[j] > 0)	return pow(1/upper + 1/lower - 4/range + 1, -1); // This is actually the INVERSE of the penalty function
	else				return 1.0;
}

/*

class MultiJointController
{
	public:
		MultiJointController(){};					// Default constructor
		
		void configure_drivers(const std::string &local_port_name,	// Configure communication with robot
					const std::string &remote_port_name,
					const std::string &_name,
					const int &number_of_joints);
					
		bool get_encoder_values();					// Get new state information
		
		double get_joint_weight(const int &j);
		
		void close() { this->driver.close();}				// Close the drivers
		
		void set_joint_trajectory(yarp::sig::Vector &position);		// Set a new joint trajectory object
		
		void move_at_speed(const yarp::sig::Vector &speed);		// Send velocity commands to each individual joint
		
		void joint_control(const double &time);				// Solve control to track trajectory					
		
		yarp::sig::Vector q, qdot;
		
	private:
		// Variables used internally in this class
		int n;								// Number of joints
		std::string name;						// Unique identifier
		yarp::sig::Vector pos, vel, acc, ctrl;				// Joint state and control vectors

		Quintic trajectory;						// Joint trajectory object
		
		
	   	// These interface with the hardware
    		yarp::dev::IControlLimits*	limits;				// Joint limits?
		yarp::dev::IControlMode*	mode;				// Sets the control mode of the motor
		yarp::dev::IEncoders*		encoder;			// Joint position values (in degrees)
		yarp::dev::IVelocityControl*	controller;			// Motor-level velocity controller
		yarp::dev::PolyDriver		driver;				// Device driver
		
};										// Semicolon needed after class declaration

/******************** Get new joint encoder information ********************
bool MultiJointController::get_encoder_values()
{
	bool success = true;
	
	for(int i = 0; i < this->n; i++)
	{
		success &= this->encoder->getEncoder(i, &this->q[i]);
		success &= this->encoder->getEncoderSpeed(i, &this->qdot[i]);
	}

	if(!success) yError() << "ArmController::update_state() : Could not obtain encoder values for " << this->name + ".";
	
	return success;
}

/******************** Get the penalty term for avoiding joint limits ********************
double MultiJointController::get_joint_weight(const int &j)
{
	double qMin, qMax;
	this->limits->getLimits(j, &qMin, &qMax);					// Get the lower and upper limits for this joint
	
	double range = qMax - qMin;							// Distance between joints
	
	double upper = qMax - this->q[j];						// Distance to upper limit
	double lower = this->q[j] - qMin;						// Distance from lower limit
	
	double dpdq = pow(upper,-2) - pow(lower, -2);					// Partial derivative of the penalty function
	
	if(dpdq*this->qdot[j] > 0)	return pow(1/upper + 1/lower - 4/range + 1, -1); // This is actually the INVERSE of the penalty function
	else				return 1.0;
}

/******************** Set a new joint trajectory ********************
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
		double qMin, qMax;						// Upper and lower joint limits
		for(int i = 0; i < this->n; i++)
		{
			this->limits->getLimits(i, &qMin, &qMax);		// Get the limits for ith joint
			if(position[i] > qMax)	position[i] = qMax - 0.01;	// Just under the joint limit
			if(position[i] < qMin)	position[i] = qMin + 0.01;	// Just above the joint limit
		}
		
		// Compute optimal time scaling
		double vMin, vMax;						// Speed limits
		double dt, dq;							// Distance in time, space
		double endTime;							// To be computed here
		
		get_encoder_values();						// Get the current joint state
		for(int i = 0; i < this->n; i++)
		{
			dq = position[i] - this->q[i];				// Distance to target
			this->limits->getVelLimits(i, &vMin, &vMax);		// Get velocity limits for ith joint
			if(dq >= 0)	dt = (15*dq)/(8*vMax);			// Time to cover distance at max speed			
			else		dt = (15*dq)/(8*vMin);					
			if(dt > endTime) endTime = dt;				// Update based on maximum time
		}
		endTime = 2.0;							// Make the time a bit longer
		
		this->trajectory = Quintic(this->q, position, 0, endTime);	// New trajectory
	}	
}

/******************** Solve the trajectory tracking control ********************
void MultiJointController::joint_control(const double &time)
{
	this->trajectory.get_state(this->pos, this->vel, this->acc, time);		// Get the desired state from the trajectory object
	
	this->ctrl = vel + 0.1*(this->pos - this->q);					// Feedforward + feedback control
	
	move_at_speed(this->ctrl);							// Send commands to joints
}

/******************** Send velocity commands to each individual joint ********************
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

/******************** As it says on the label ********************
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
			this->controller->setRefAcceleration(i, std::numeric_limits<double>::max()); 	// CHANGE THIS?
			this->controller->velocityMove(i, 0.0);						// Ensure initial velocity is zero
		}
	}
	
	// Now try and obtain joint limits
	if(!this->driver.view(this->limits))
	{
		yError() << "Unable to obtain joint limits for" << this->name + ".";
		totalSuccess = false;
	}
	
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
				break;								// This should break the loop
			}
			if(i == 5)
			{
				yError() << "Could not obtain encoder values for" << this->name << "in 5 attempts.";
				totalSuccess = false;
			}
			yarp::os::Time::delay(0.05);						// wait a little bit and try again
		}
	}
	if(totalSuccess) yInfo() << "Successfully configured drivers for the" << this->name + ".";
}
*/
