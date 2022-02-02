/*
*	Communication with motor controllers on the iCub robot.
*/

#ifndef JOINTINTERFACE_H_
#define JOINTINTERFACE_H_

#include <iDynTree/Core/VectorDynSize.h>						// iDynTree::VectorDynSize
#include <string>									// std::string
#include <vector>									// std::vector
#include <yarp/dev/ControlBoardInterfaces.h>						// I don't know what this does exactly...
#include <yarp/dev/PolyDriver.h>							// ... or this...
#include <yarp/os/Property.h>								// ... or this.

class JointInterface
{
	public:
		JointInterface(const std::vector<std::string> &jointList);
		
		bool read_encoders();							// Update the joint states internally
		
		bool send_torque_commands(const iDynTree::VectorDynSize &tau);
		
	protected:
		iDynTree::VectorDynSize q, qdot, qMin, qMax, vLim;
	private:
		bool isValid = true;							// Won't do calcs if false
		int n;
		
	   	// These interface with the hardware
		yarp::dev::IControlLimits*	limits;					// Joint limits?
		yarp::dev::IControlMode*	mode;					// Sets the control mode of the motor
		yarp::dev::IEncoders*		encoders;				// Joint position values (in degrees)
		yarp::dev::ITorqueControl*	tControl;
		yarp::dev::IVelocityControl*	vControl;				// Motor-level velocity controller
		yarp::dev::PolyDriver		driver;					// Device driver
	
};											// Semicolon needed after class declaration

/******************** Constructor ********************/
JointInterface::JointInterface(const std::vector<std::string> &jointList)
				: n(jointList.size())
{
	// Resize arrays, vectors
	q.resize(this->n);
	qdot.resize(this->n);
	qMin.resize(this->n);
	qMax.resize(this->n);
	vLim.resize(this->n);
	
	// Open up device drivers
	yarp::os::Property options;									
	options.put("device", "remotecontrolboardremapper");
	
	options.addGroup("axesNames");
	yarp::os::Bottle & bottle = options.findGroup("axesNames").addList();
	for(int i = 0; i < jointList.size(); i++) bottle.addString(jointList[i].c_str());
		
	yarp::os::Bottle remoteControlBoards;
	yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();
	remoteControlBoardsList.addString("/icubSim/torso");
	remoteControlBoardsList.addString("/icubSim/left_arm");
	remoteControlBoardsList.addString("/icubSim/right_arm");
	remoteControlBoardsList.addString("/icubSim/left_leg");
	remoteControlBoardsList.addString("/icubSim/right_leg");
//	remoteControlBoardsList.addString("/icubSim/neck"); 
	
	options.put("remoteControlBoards", remoteControlBoards.get(0));
	options.put("localPortPrefix", "/local");
	
	yarp::os::Property &remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
			    remoteControlBoardsOpts.put("writeStrict", "on");
	
	if(!this->driver.open(options))
	{
		std::cerr << "[ERROR] [JOINTINTERFACE] Constructor: Could not open device driver." << std::endl;
		this->isValid &= false;
	}
	else
	{
		// Try and configure the joint controllers
//		if(!this->driver.view(this->tControl))
		if(!this->driver.view(this->vControl))
		{
			std::cout << "[ERROR] [JOINTINTERFACE] Constructor: Unable to configure the motor controllers." << std::endl;
			this->isValid &= false;
		}
		else if(!this->driver.view(this->mode))
		{
			std::cerr << "[ERROR] [JOINTINTERFACE] Unable to configure the control mode." << std::endl;
			this->isValid &= false;
		}
		else
		{
			// Success! Set control mode properties for all joints
			for(int i = 0; i < this->n; i++)
			{
				this->mode->setControlMode(i, VOCAB_CM_VELOCITY);				// Put it in velocity mode
				this->vControl->setRefAcceleration(i, std::numeric_limits<double>::max()); 	// Allow maximum joint acceleration
				this->vControl->velocityMove(i, 0.0);						// Ensure initial velocity of zero
				
				// this->mode->setControlMode(i, torque?)
			}
		}
		
		// Now try and obtain the joint limits
		if(!this->driver.view(this->limits))
		{
			std::cerr << "[ERROR] [JOINTINTERFACE] Unable to obtain joint limits." << std::endl;
			this->isValid &= false;
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
			std::cerr << "[ERROR] [JOINTINTERFACE] Unable to configure the encoders." << std::endl;
			this->isValid &= false;
		}
		else
		{
			double temp[this->n];							// Temporary storage
			
			for(int i = 0; i < 5; i++)
			{
				if(this->encoders->getEncoders(temp)) break;
				
				if(i == 5)
				{
					std::cerr << "[ERROR] [JOINTINTERFACE] Could not obtain encoder values for in 5 attempts." << std::endl;
					this->isValid &= false;
				}
				yarp::os::Time::delay(0.05);
			}
			
			for(int i = 0; i < this->n; i++) this->q[i] = temp[i]*M_PI/180;
		}
	}
	
	if(this->isValid)	std::cout << "[INFO] [JOINTINTERFACE] Successfully configured the drivers." << std::endl;
	else			this->driver.close();	
}

/******************** Get new joint encoder information ********************/
bool JointInterface::read_encoders()
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

	if(!success) std::cerr << "[ERROR] [JOINTINTERFACE] Could not obtain new encoder values." << std::endl;
	
	return success;
}

/******************** Send torque commands to the joints ********************/
bool JointInterface::send_torque_commands(const iDynTree::VectorDynSize &tau)
{
	if(tau.size() > this->n)
	{
		std::cerr << "[WARNING] [JOINTINERFACE] send_torque_commands() : Input vector has "
			<< tau.size() << " elements but there are only " << this->n << " joints." << std::endl;
			
		for(int i = 0; i < this->n; i++) this->tControl->setRefTorque(i, tau[i]);
		
		return false;
	}
	else
	{
		for(int i = 0; i < this->n; i++) this->tControl->setRefTorque(i, tau[i]);
		
		return true;
	}
}

#endif
