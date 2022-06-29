    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     A class for interfacing with the joint motors on the iCub                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef JOINTINTERFACE_H_
#define JOINTINTERFACE_H_

#include <iDynTree/Core/VectorDynSize.h>                                                            // iDynTree::VectorDynSize
#include <iostream>                                                                                 // std::cerr, std::cout
#include <math.h>                                                                                   // M_PI
#include <string>                                                                                   // std::string
#include <vector>                                                                                   // std::vector
#include <yarp/dev/ControlBoardInterfaces.h>                                                        // I don't know what this does exactly...
#include <yarp/dev/PolyDriver.h>                                                                    // ... or this...
#include <yarp/os/Property.h>                                                                       // ... or this.

class JointInterface
{
	public:
		JointInterface(const std::vector<std::string> &jointList);                           // Constructor
		
		bool activate_control();
		bool get_joint_state(iDynTree::VectorDynSize &q, iDynTree::VectorDynSize &qdot);    // Get the joint state as iDynTree objects
		bool read_encoders();                                                               // Update the joint states internally
		bool send_torque_commands(const iDynTree::VectorDynSize &command);                  // Send control to the joint motors
		void close();
        
        protected:
		int n;                                                                              // Number of joints being controlled	
		std::vector<double> pMin, pMax, vMax;                                               // Position and velocity limits for each joint
                                
	private:
		// Properties
		bool isValid = false;                                                               // Won't do anything if false	
		std::vector<double> pos, vel;                                                       // Joint position and velocity limits
		
	   	// These interface with the hardware on the robot itself
		yarp::dev::IControlLimits*   limits;                                                // Joint limits?
		yarp::dev::IControlMode*     mode;                                                  // Sets the control mode of the motor
		yarp::dev::IEncoders*        encoders;                                              // Joint position values (in degrees)
		yarp::dev::ITorqueControl*   controller;
		yarp::dev::PolyDriver        driver;                                                // Device driver
	
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
JointInterface::JointInterface(const std::vector<std::string> &jointList) :
                               n(jointList.size())
{
	// Resize std::vector objects
	this->pos.resize(this->n);
	this->vel.resize(this->n);
	this->pMin.resize(this->n);
	this->pMax.resize(this->n);
	this->vMax.resize(this->n);
	
	/************************** I copied this code from elsewhere *****************************/
	
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
			    
	/******************************************************************************************/
	
	if(not this->driver.open(options))
	{
		std::cerr << "[ERROR] [JOINT INTERFACE] Constructor: "
			  << "Could not open device driver." << std::endl;
	}
	else
	{	
		if(not this->driver.view(this->controller))
		{
			std::cerr << "[ERROR] [JOINT INTERFACE] Constructor: "
			          << "Unable to configure the controller for the joint motors." << std::endl;
		}
		else if(not this->driver.view(this->mode))
		{
			std::cerr << "[ERROR] [JOINT INTERFACE] Constructor: "
				  << "Unable to configure the control mode." << std::endl;
		}
		else if(not this->driver.view(this->limits))
		{
			std::cerr << "[ERROR] [JOINT INTERFACE] Constructor: "
			          << "Unable to obtain the joint limits." << std::endl;
		}
		else
		{
			// Opened the motor controllers, so get the joint limits
			for(int i = 0; i < this->n; i++)
			{
				double notUsed;
				this->limits->getLimits(i, &this->pMin[i], &this->pMax[i]);         // Get the position limits
				this->limits->getVelLimits(i, &notUsed, &this->vMax[i]);            // Get the velocity limits (assume vMin = -vMax)
				
				// Convert from degrees to radians
				this->pMin[i] *= M_PI/180;
				this->pMax[i] *= M_PI/180;
				this->vMax[i] *= M_PI/180;
			}
			
			// Finally, configure the encoders
			if(not this->driver.view(this->encoders))
			{
				std::cerr << "[ERROR] [JOINT INTERFACE] Constructor: "
					  << "Unable to configure the encoders." << std::endl;
			}
			else
			{
				double temp[this->n];                                               // Temporary array to hold encoder values
				
				// Make 5 attempts to read the encoders
				for(int i = 0; i < 5; i++)
				{
					if(not this->encoders->getEncoders(temp) and i == 4)
					{
						std::cerr << "[ERROR] [JOINT INTERFACE] Constructor: "
						          << "Could not obtain encoder values in 5 attempts." << std::endl;
					}
					else
					{
						this->isValid = true;                               // Success! We made it to the end.
						
						for(int j = 0; j < this->n; j++) this->pos[i] = temp[i]*M_PI/180; // Assign initial joint values
						
						break;
					}
				}
			}
		}					
	}
	
	if(this->isValid) std::cout << "[INFO] [JOINT INTERFACE] Constructor: "
			            << "Successfully configured the drivers." << std::endl;
					  
	else              this->driver.close();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Set control mode and allow commands to be sent                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool JointInterface::activate_control()
{
	if(not this->isValid)
	{
		std::cerr << "[ERROR] [JOINT INTERFACE] activate_control(): "
		          << "Something went wrong during the construction of this object. "
		          << "Could not activate joint control." << std::endl;
		          
		return false;
	}
	else
	{
		for(int i = 0; i < this->n; i++) this->mode->setControlMode(i, VOCAB_CM_TORQUE);
		
		return true;
	}  
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Get the joint state as iDynTree::VectorDynSize objects                       //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool JointInterface::get_joint_state(iDynTree::VectorDynSize &q, iDynTree::VectorDynSize &qdot)
{
	if(not this->isValid)
	{
		std::cerr << "[ERROR] [JOINT INTERFACE] get_joint_state(): "
			  << "Something went wrong during the construction of this object. "
			  << "Could not obtain the joint state." << std::endl;
			  
		return false;
	}
	else if(q.size() != this->n or qdot.size() != this->n)
	{
		std::cout << "[ERROR] [JOINT INTERFACE] get_joint_state(): "
			  << "Input vectors are not the correct size! "
			  << "There are " << this->n << " joints in this model, but "
			  << "the position vector had " << q.size() << " elements and "
			  << "the velocity vector had " << qdot.size() << " elements." << std::endl;
			  
		return false;
	}
	else
	{
		for(int i = 0; i < this->n; i++)
		{
			q[i]    = this->pos[i];
			qdot[i] = this->vel[i];
		}
		
		return true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get new joint state information from the encoders                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool JointInterface::read_encoders()
{
	bool success = true;
	for(int i = 0; i < this->n; i++)
	{
		success &= this->encoders->getEncoder(i, &this->pos[i]);                            // Read the position
		success &= this->encoders->getEncoderSpeed(i, &this->vel[i]);                       // Read the velocity
		
		this->pos[i] *= M_PI/180;                                                           // Convert to rad
		this->vel[i] *= M_PI/180;                                                           // Convert to rad/s
	}

	if(not success) std::cerr << "[ERROR] [JOINT INTERFACE] read_encoders(): "
			          << "Could not obtain new encoder values." << std::endl;
	
	return success;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Send commands to the joints                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool JointInterface::send_torque_commands(const iDynTree::VectorDynSize &command)
{
	if(not this->isValid)
	{
		std::cerr << "[ERROR] [JOINT INTERFACE] send_joint_commands(): "
		          << "There was a problem during construction of this object. "
		          << "Joint commands not sent." << std::endl;
		
		return false;
	}
	else
	{
		int numJoints = command.size();
		
		if(command.size() > this->n)
		{
			std::cerr << "[WARNING] [JOINT INTERFACE] send_joint_commands(): "
			          << "Command vector had " << command.size() << " elements "
			          << "but there are only " << this->n << " joints in this object! "
			          << "Ignoring surplus commands..." << std::endl;
			          
			numJoints = this->n;
		}
	
		for(int i = 0; i < numJoints; i++) this->controller->setRefTorque(i,command[i]);
		
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

#endif
