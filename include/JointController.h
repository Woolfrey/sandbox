/*
*	Communication with motor controllers on the iCub robot.
*/

#ifndef JOINTCONTROLLER_H_
#define JOINTCONTROLLER_H_

#include <iDynTree/Core/VectorDynSize.h>						// iDynTree::VectorDynSize
#include <string>									// std::string
#include <vector>									// std::vector
#include <yarp/dev/ControlBoardInterfaces.h>						// I don't know what this does exactly...
#include <yarp/dev/PolyDriver.h>							// ... or this...
#include <yarp/os/Property.h>								// ... or this.

class JointController
{
	public:
		JointController() {}							// Empty constructor
		
		bool configure_drivers(const std::string &localPortName,	
					const std::string &remotePortName,
					const std::string &_name,
					const int &numberOfJoints);
					
		bool read_encoders();			
		
		iDynTree::VectorDynSize get_joint_positions() const {return this->q;} 	// As it says on the label
		iDynTree::VectorDynSize get_joint_velocities() const {return this->qdot;} // As it says on the label
		
		void close() {this->driver.close();}					// Close the driver(s)
		
		
	private:
		bool isConfigured = true;						// Won't do anything is this is false
		iDynTree::VectorDynSize q, qdot, qMin, qMax, vLim;
		int n;									// Number of joints
		std::string name;							// For identification purposes
//		std::vector<double> q, qdot, qMin, qMax, vLim;				// Joint positions & velocities
		
	   	// These interface with the hardware
    		yarp::dev::IControlLimits*	limits;					// Joint limits?
		yarp::dev::IControlMode*	mode;					// Sets the control mode of the motor
		yarp::dev::IEncoders*		encoders;				// Joint position values (in degrees)
		yarp::dev::IVelocityControl*	controller;				// Motor-level velocity controller
		yarp::dev::PolyDriver		driver;					// Device driver

};											// Semicolon needed after a class declaration

/******************** Configure the device drivers to communicate with the robot ********************/
bool JointController::configure_drivers(const std::string &localPortName,
					const std::string &remotePortName,
					const std::string &_name,
					const int &numberOfJoints)
{
	// Assign input values
	this->name = _name;
	this->n = numberOfJoints;
	
	// Resize vector objects according to number of joints
	this->q.resize(this->n);
	this->qdot.resize(this->n);
	this->qMin.resize(this->n);
	this->qMax.resize(this->n);
	this->vLim.resize(this->n);
	
	// First configure the main device driver
	yarp::os::Property options;
	options.put("device", "remote_controlboard");
	options.put("local", localPortName);
	options.put("remote", remotePortName);
	
	this->driver.open(options);
	if(!this->driver.isValid())
	{
		std::cerr << "[ERROR] [JOINTCONTROLLER] Unable to configure the device driver for " << this->name << "." << std::endl;
		this->isConfigured = false;
	}
	
	// Try and configure the joint controllers
	if(!this->driver.view(this->controller))
	{
		std::cerr << "[ERROR] [JOINTCONTROLLER] Unable to configure the controller for " << this->name << "." << std::endl;
		this->isConfigured = false;
	}
	else if(!this->driver.view(this->mode))
	{
		std::cerr << "[ERROR] [JOINTCONTROLLER] Unable to configure the control mode for " << this->name << "." << std::endl;
		this->isConfigured = false;
	}
	else
	{
		// Success! Set control mode properties for all joints
		for(int i = 0; i < this->n; i++)
		{
			this->mode->setControlMode(i, VOCAB_CM_VELOCITY);				// Put it in velocity mode
			this->controller->setRefAcceleration(i, std::numeric_limits<double>::max()); 	// Allow maximum joint acceleration
			this->controller->velocityMove(i, 0.0);						// Ensure initial velocity of zero
		}
	}
	
	// Now try and obtain the joint limits
	if(!this->driver.view(this->limits))
	{
		std::cerr << "[ERROR] [JOINTCONTROLLER] Unable to obtain joint limits for " << this->name << "." << std::endl;
		this->isConfigured = false;
	}
	else
	{
		for(int i = 0; i < this->n; i++)
		{
			double notUsed;
			this->limits->getLimits(i, &this->qMin[i], &this->qMax[i]);
			this->limits->getVelLimits(i, &notUsed, &this->vLim[i]);
			
			// Convert from degrees to radians
			this->qMin[i] *= M_PI/180;
			this->qMax[i] *= M_PI/180;
			this->vLim[i] *= M_PI/180;
		}
	}
	
	// Finally, configure the encoders
	if(!this->driver.view(this->encoders))
	{
		std::cerr << "[ERROR] [JOINTCONTROLLER] Unable to configure the encoders for " << this->name << "." << std::endl;
		this->isConfigured *= false;
	}
	else
	{
		double temp[this->n];							// Temporary storage
		
		for(int i = 0; i < 5; i++)
		{
			if(this->encoders->getEncoders(temp)) break;
			
			if(i == 5)
			{
				std::cerr << "[ERROR] [JOINTCONTROLLER] Could not obtain encoder values for "
					<< this->name << " in 5 attempts." << std::endl;
				
				this->isConfigured = false;
			}
			yarp::os::Time::delay(0.05);
		}
		
		for(int i = 0; i < this->n; i++) this->q[i] = temp[i]*M_PI/180;
	}	
	
	if(this->isConfigured) std::cout << "[INFO] [JOINTCONTROLLER] Successfully configured the drivers for " << this->name << "." << std::endl;
	else this->driver.close();
	
	return this->isConfigured;
}

/******************** Get new joint encoder information ********************/
bool JointController::read_encoders()
{
	bool success = true;
	for(int i = 0; i < this->n; i++)
	{
		success &= this->encoders->getEncoder(i, &this->q[i]);			// Read the position
		success &= this->encoders->getEncoderSpeed(i, &this->qdot[i]);		// Read the velocity
		
		this->q[i] *= M_PI/180;							// Convert to rad
		this->qdot[i] *= M_PI/180;						// Convert to rad/s
		
		// Sensor noise can give readings above or below limits so cap them
		if(this->q[i] > this->qMax[i]) this->q[i] = this->qMax[i];
		if(this->q[i] < this->qMin[i]) this->q[i] = this->qMin[i];
	}

	if(!success) std::cerr << "[ERROR] [JOINTCONTROLLER] Could not obtain new encoder values for " << this->name << "." << std::endl;
	
	return success;
}

#endif
