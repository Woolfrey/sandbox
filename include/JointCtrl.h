#include <Quintic.h>								// Custom trajectory class
#include <yarp/dev/ControlBoardInterfaces.h>					// Communicates with motor controllers on the robot?
#include <yarp/dev/PolyDriver.h>						// Device driver class
#include <yarp/os/LogStream.h>							// yarp::Info() and the like
#include <yarp/os/Property.h>							// Don't know exactly what this does

class JointCtrl
{
	public:
		// Constructor
		JointCtrl() {};
		
		// General Functions
		bool configure_drivers(const std::string &local_port_name,	// Configure drivers for robot hardware
				const std::string &remote_port_name,
				const std::string &_name,
				const int &number_of_joints);			
		void close() { this->driver.close();}				// Close the driver
		bool move_at_speed(const yarp::sig::Vector &speed);		// Set velocity command on motor drivers
		void track_joint_trajectory(const double &time);		// Track the internal trajectory object
		
		// Set Functions
		bool set_frequency(const double &freq);
		bool set_joint_trajectory(yarp::sig::Vector &target);		// Set a new joint trajectory object
			
		// Get Functions
		bool read_encoders();						// Read position, velocity of the joints
		bool get_speed_limits(const int &i, double &lower, double &upper); // Get the velocity constraints for a single joint
		double get_joint_weight(const int &i);				// Used in joint limit avoidance
		yarp::sig::Vector get_joint_positions() {return this->q;}	// As it says
		yarp::sig::Vector get_joint_velocities() {return this->qdot;}	// As it says
		
	protected:
		double hertz = 100;						// Control frequency
		int n;								// Number of joints
		std::string name;						// Identifies this object
		
	private:
		double upper, lower, penalty, dpdq;				// Used in joint limit avoidance
				
		Quintic trajectory;						// Joint-level trajectory generator
		
		yarp::sig::Vector q, qdot;					// Joint position and velocity
		yarp::sig::Vector qMin, qMax, range, vLim;			// Position and velocity limits
		yarp::sig::Vector pos, vel, acc;				// Desired state for control purposes
		
	   	// These interface with the hardware
    		yarp::dev::IControlLimits*	limits;				// Joint limits?
		yarp::dev::IControlMode*	mode;				// Sets the control mode of the motor
		yarp::dev::IEncoders*		encoder;			// Joint position values (in degrees)
		yarp::dev::IVelocityControl*	controller;			// Motor-level velocity controller
		yarp::dev::PolyDriver		driver;				// Device driver
		
};										// Semicolon needed after a class declaration

/******************** Set the control frequency (usedf or internal calcs) ********************/
bool JointCtrl::set_frequency(const double &freq)
{
	if(freq > 0)
	{
		this->hertz = freq;
		return true;
	}
	else
	{
		yError("JointCtrl::set_control_frequency() : Frequency cannot be negative!");
		this->hertz = 100;
		return false;
	}
}

/******************** As it says on the label ********************/
bool JointCtrl::configure_drivers(const std::string &local_port_name,
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
	this->range.resize(this->n);							// Range between joint limits
	this->vLim.resize(this->n);							// Joint speed limit
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
			this->mode->setControlMode(i, VOCAB_CM_VELOCITY);			// Set the motor in velocity mode
			this->controller->setRefAcceleration(i, std::numeric_limits<double>::max()); // CHANGE THIS?
			this->controller->velocityMove(i, 0.0);					// Ensure initial velocity is zero
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
		double vMin;									// Not really used
		for(int i = 0; i < this->n; i++)
		{
			this->limits->getLimits(i, &this->qMin[i], &this->qMax[i]);
			this->limits->getVelLimits(i, &vMin, &this->vLim[i]);			// vMin is always zero
		}
		
		this->qMin *= M_PI/180;								// Convert to radians
		this->qMax *= M_PI/180;
		this->range = this->qMax - this->qMin;						// This is used in joint limit avoidance
		this->vLim *= M_PI/180;								// Convert to rad/s
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
	
	return totalSuccess;
}

/******************** Set a new joint trajectory ********************/
bool JointCtrl::set_joint_trajectory(yarp::sig::Vector &target)
{
	// Check the inputs are sound
	if(target.size() > this->n)
	{
		yError() << "JointCtrl::set_joint_trajectory() : Length of input vector is greater than the number of joints!";
		return false;
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
		double dt, dq;
		double endTime = 2.0;
		
		for(int i = 0; i < this->n; i++)
		{
			dq = abs(target[i] - this->q[i]);				// Distance to target
			if(dq > 0)	dt = (15*dq)/(8*this->vLim[i]);			// Optimal time scaling at peak velocity
			if(dt > endTime) endTime = dt;
		}
		
		this->trajectory = Quintic(this->q, target, 0.0, endTime);
		return true;
	}
}

/******************** Send velocity commands to each individual joint ********************/
bool JointCtrl::move_at_speed(const yarp::sig::Vector &speed)
{
	if(speed.size() != this->n)
	{
		yError("JointCtrl::move_at_speed() : Length of input vector does not match the number of joints!");
		return false;
	}
	else
	{
		for(int i = 0; i < this->n; i++) this->controller->velocityMove(i,speed[i]*180/M_PI); // Convert from rad/s to deg/s
		return true;
	}
}

/******************** Return feedback control to track trajectory object ********************/
void JointCtrl::track_joint_trajectory(const double &time)
{
	this->trajectory.get_state(this->pos, this->vel, this->acc, time);		// Get the desired state from the trajectory object
	move_at_speed(this->vel + 1.0*(this->pos - this->q));				// Send commands to the motors
}

/******************** Get new joint encoder information ********************/
bool JointCtrl::read_encoders()
{
	bool success = true;
	
	for(int i = 0; i < this->n; i++)
	{
		success &= this->encoder->getEncoder(i, &this->q[i]);		// Read the position
		success &= this->encoder->getEncoderSpeed(i, &this->qdot[i]);	// Read the velocity
	}
	
	this->q *= M_PI/180;							// Convert to radians
	this->qdot *= M_PI/180;							// Convert to rad/s
	
	// Sensor noise can make the values go over the limits.
	for(int i = 0; i < this->n; i++)
	{
		if(this->q[i] > this->qMax[i])		this->q[i] = this->qMax[i];
		else if(this->q[i] < this->qMin[i])	this->q[i] = this->qMin[i];
	}

	if(!success) yError() << "ArmController::update_state() : Could not obtain encoder values for " << this->name + ".";
	
	return success;
}

/******************** Get the min, max speed for each joint to avoid limits ********************/
bool JointCtrl::get_speed_limits(const int &i, double &lower, double &upper)
{
	double temp = this->hertz*(this->qMax[i] - this->q[i]);				// Maximum speed to reach upper joint limit

	if(this->vLim[i] > temp)	upper = temp;
	else				upper = this->vLim[i];
	
	temp = this->hertz*(this->qMin[i] - this->q[i]);				// Minimum speed to reach lower limit
	
	if(-1*this->vLim[i] < temp)	lower = temp;
	else				lower = -1*this->vLim[i];
	
	return true; // Not sure what to do with this
}

/******************** Get the penalty term for avoiding joint limits ********************/
double JointCtrl::get_joint_weight(const int &i)
{
	this->upper = this->qMax[i] - this->q[i];					// Distance to upper limit
	this->lower = this->q[i] - this->qMin[i];					// Distance to lower limit
	this->dpdq = (pow(this->range[i],2)*(2*this->q[i] - this->qMax[i] - this->qMin[i]))/
			(4*pow(this->upper,2)*pow(this->lower,2));
	
	if(dpdq*this->qdot[i] > 0)							// Moving toward a joint limit
	{
		this->penalty = pow(this->range[i],2)/(4*this->upper*this->lower);
		if(this->penalty < 0)
		{
			yError() << "Penalty term for joint " << i + 1 << " of the " << this->name << ": " << this->penalty
			<< " Joint position: " << this->qMin[i] << " " << this->q[i] << " " << this->qMax[i];
			return 0;							// Something went wrong and the joint is outside its limits?
		}
		else	return 1/this->penalty;						// Returns the INVERSE when moving toward a limit
	}
	else		return 1.0;							// Free movement away from the limits
}

