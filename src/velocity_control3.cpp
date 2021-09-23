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
	Quintic();
	
	
	private:
	
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
				this->controller->getAxes(&this->n);				// NO. OF JOINTS IS SET HERE
				
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
	ArmController left_arm(local, remote, name);						// Create and configure arm controller object
	
	// Configure the right arm
	local = "/local/right";
	remote = "/icubSim/right_arm";
	name = "right_arm";
	ArmController right_arm(local, remote, name);						// Create and configure arm controller object
	
	/*** Run a loop here to move the arms to the home position ***/
	
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
		
		port.read(input, true);							// Get any commands over the network
		command = input.toString();							// Convert to a string
		
		// Shut down the robot
		if(command == "close")
		{
			output.addString("Arrivederci");
			active = false;							// Shut down the while loop
		}
		
		// Extend out both arms
		else if(command == "receive")
		{
			output.addString("Grazie");
		}

		// Extend one arm to shake hands
		else if(command == "shake")
		{
			output.addString("Piacere");
		}

		// Wave one arm
		else if(command == "wave")
		{
			output.addString("Ciao");
		}
		
		// Unknown command
		else
		{
			output.addString("Cosa");
		}
		
		port.reply(output);								// Send response to user
	}
	
	left_arm.close();
	right_arm.close();
	
	return 0;										// No problems
}

