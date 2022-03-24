    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     A class for interfacing with the joint motors on the iCub                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JOINTINTERFACE_H_
#define JOINTINTERFACE_H_

#include <math.h>                                                                                  // M_PI
#include <string>                                                                                  // std::string
#include <vector>                                                                                  // std::vector
#include <yarp/dev/ControlBoardInterfaces.h>                                                       // I don't know what this does exactly...
#include <yarp/dev/PolyDriver.h>                                                                   // ... or this...
#include <yarp/os/Property.h>                                                                      // ... or this.

class JointInterface
{
	public:
		JointInterface(const std::vector<std::string> &jointList);

		bool activate_control();                                                           // Activate the joint control	
		bool read_encoders();                                                              // Update the joint states internally
		bool send_torque_commands(const std::vector<double> &tau);                         // As it says on the label
		void close();                                                                      // Close the device driver
		
	protected:
		int n;                                                                             // Number of joints being controlled
		std::vector<double> pos, vel;                                                      // Joint positions and velocities
		std::vector<double> pMin, pMax, vMax;                                              // Position and velocity limits
		
	private:
		bool isValid = true;                                                               // Won't do calcs if false
		
	   	// These interface with the hardware
		yarp::dev::IControlLimits*  limits;                                                // Joint limits?
		yarp::dev::IControlMode*    mode;                                                  // Sets the control mode of the motor
		yarp::dev::IEncoders*       encoders;                                              // Joint position values (in degrees)
		yarp::dev::ITorqueControl*  controller;
		yarp::dev::PolyDriver       driver;                                                // Device driver
	
};                                                                                                 // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
JointInterface::JointInterface(const std::vector<std::string> &jointList):
                               n(jointList.size())
{
	// Resize std::vector objects
	this->pos.resize(this->n);
	this->vel.resize(this->n);
	this->pMin.resize(this->n);
	this->pMax.resize(this->n);
	this->vMax.resize(this->n);
	
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
		std::cerr << "[ERROR] [JOINTINTERFACE] Constructor: "
			  << "Could not open device driver." << std::endl;
		this->isValid &= false;
	}
	else
	{
		// Try and configure the joint controllers
		if(!this->driver.view(this->controller))
		{
			std::cout << "[ERROR] [JOINTINTERFACE] Constructor: "
				  << "Unable to configure the motor controllers." << std::endl;
			this->isValid &= false;
		}
		else if(!this->driver.view(this->mode))
		{
			std::cerr << "[ERROR] [JOINTINTERFACE] Constructor: "
				  << "Unable to configure the control mode." << std::endl;
			this->isValid &= false;
		}
		
		// Now try and obtain the joint limits
		if(!this->driver.view(this->limits))
		{
			std::cerr << "[ERROR] [JOINTINTERFACE] Constructor: "
				  << "Unable to obtain joint limits." << std::endl;
			this->isValid &= false;
		}
		else
		{
			for(int i = 0; i < this->n; i++)
			{
				double notUsed;
				this->limits->getLimits(i, &this->pMin[i], &this->pMax[i]);
				this->limits->getVelLimits(i, &notUsed, &this->vMax[i]);
				
				// Convert from degrees to radians
				this->pMin[i] *= M_PI/180;
				this->pMax[i] *= M_PI/180;
				this->vMax[i] *= M_PI/180;
			}
		}
		
		// Finally, configure the encoders
		if(!this->driver.view(this->encoders))
		{
			std::cerr << "[ERROR] [JOINTINTERFACE] Constructor: "
				  << "Unable to configure the encoders." << std::endl;
			this->isValid &= false;
		}
		else
		{
			double temp[this->n];							   // Temporary storage
			
			for(int i = 0; i < 5; i++)
			{
				if(this->encoders->getEncoders(temp)) break;
				
				if(i == 5)
				{
					std::cerr << "[ERROR] [JOINTINTERFACE] Constructor: "
						  << "Could not obtain encoder values for in 5 attempts." << std::endl;
					this->isValid &= false;
				}
				yarp::os::Time::delay(0.05);
			}
			
			for(int i = 0; i < this->n; i++) this->pos[i] = temp[i]*M_PI/180;
		}
	}
	
	if(this->isValid)	std::cout << "[INFO] [JOINTINTERFACE] Constructor: "
					  << "Successfully configured the drivers." << std::endl;
	else			this->driver.close();	
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get new joint state information from the encoders                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool JointInterface::read_encoders()
{
	bool success = true;
	for(int i = 0; i < this->n; i++)
	{
		success &= this->encoders->getEncoder(i, &this->pos[i]);                           // Read the position
		success &= this->encoders->getEncoderSpeed(i, &this->vel[i]);                      // Read the velocity
		
		this->pos[i] *= M_PI/180;                                                          // Convert to rad
		this->vel[i] *= M_PI/180;                                                          // Convert to rad/s
		
		// Sensor noise can give readings above or below limits so cap them
		if(this->pos[i] > this->pMax[i]) this->pos[i] = this->pMax[i];
		if(this->pos[i] < this->pMin[i]) this->pos[i] = this->pMin[i];
	}

	if(!success) std::cerr << "[ERROR] [JOINTINTERFACE] read_encoders(): "
			       << "Could not obtain new encoder values." << std::endl;
	
	return success;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Send torque commands to the joints                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool JointInterface::send_torque_commands(const std::vector<double> &tau)
{
	if(!this->isValid)
	{
		std::cerr << "[ERROR] [JOINTINTERFACE] send_torque_commands(): "
			  << "There was a problem with the construction of this object. "
			  << "Joint commands not sent." << std:: endl;
		return false;
	}
	else if(tau.size() > this->n)
	{
		std::cerr << "[WARNING] [JOINTINERFACE] send_torque_commands(): "
			  << "Input vector has " << tau.size() << " elements but there are only "
			  << this->n << " joints." << std::endl;
			
		for(int i = 0; i < this->n; i++) this->controller->setRefTorque(i, tau[i]);
		return false;
	}
	else
	{
		for(int i = 0; i < tau.size(); i++) this->controller->setRefTorque(i, tau[i]);
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Close the device interfaces on the robot                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
void JointInterface::close()
{
	if(this->isValid)
	{
		// Put it in to position mode to lock the joints
		for(int i = 0; i < this->n; i++) this->mode->setControlMode(i, VOCAB_CM_POSITION);
	}
	this->driver.close();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Set control mode and allow commands to be sent                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool JointInterface::activate_control()
{
	if(this->isValid)
	{
		for(int i = 0; i < this->n; i++) this->mode->setControlMode(i, VOCAB_CM_TORQUE);
		return true;
	}
	else
	{
		std::cerr << "[ERROR] [JOINTINTERFACE] activate_control() : "
			  << "There was a problem during the construction of this object. "
			  << "Joint control not activated." << std::endl;
		return false;
	}
}

#endif
