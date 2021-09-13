// #include <cstdio>							
#include <cmath>							// round() function
#include <iostream>							// Print to console
#include <string>							// std::string object
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>						// yarp::os::Bottle for simple communications
#include <yarp/os/Network.h>						// Non so?
#include <yarp/os/Property.h>						// Don't know exactly what this does
#include <yarp/os/RpcServer.h>					// Allows communication between ports
#include <yarp/sig/Vector.h>						// Linear algebra for signal processing

// Global variables
int n;
yarp::sig::Vector joint_position;					// Joint position information
yarp::sig::Vector target;


int main(int argc, char *argv[])
{
	yarp::os::Network yarp;					// Set up YARP
	
	// Configure communication
	yarp::os::RpcServer port;					// Create a port with a Bottle object as the input
	port.open("/command");						// Open the port with the name /command
	yarp::os::Bottle input;					// Store information from user input
	yarp::os::Bottle output;					// Store information to send to user
	std::string answer;						// Response message
	
	// Configure the robot
	yarp::os::Property options;
 	options.put("device", "remote_controlboard");
 	options.put("local", "/test/client");				// Local port names
 	options.put("remote", "/icubSim/right_arm");			// Where we want to connect to
 	yarp::dev::PolyDriver robotDevice(options);			// Create device driver for the specified robot
  	if(!robotDevice.isValid())					
 	{
 		std::printf("Device not available. Here are the known devices:\n");		// Inform user
 		std::printf("%s", yarp::dev::Drivers::factory().toString().c_str());	
 		return 0;									// Shut down
 	}
 	
 	// Configure controller
 	yarp::dev::IPositionControl *controller;			// Requires a pointer?
 	if(!robotDevice.view(controller)) std::cout << "Problems acquiring controller interface." << std::endl;
 	controller->getAxes(&n);					// Get the number of motors that can be controlled
 	target.resize(n);
	yarp::sig::Vector temp;
	temp.resize(n);
 	for(int i = 0; i < n; i++)					// Iterate through each joint...
 	{
 		controller->setRefSpeed(i,10.0);			// Set reference speed (deg/s)
 		temp[i] = 50;
 	}		
 	controller->setRefAccelerations(temp.data());			// Set reference accelerations (deg/s^2) (WHY LIKE THIS?)
 	
 	// Configure encoder information
 	yarp::dev::IEncoders *encoders;				// Needs a pointer for some reason?
 	if(!robotDevice.view(encoders)) std::cout << "Problems acquiring encoder interface." << std::endl;
 	joint_position.resize(n);					// Joint position information to be received from the encoders
 	
	// Receive commands and respond appropriately	
	bool run = true;
	while(run)
	{
		output.clear();							// Clear any previous information
		
		port.read(input,true);							// Get the input command
		std::string command = input.toString();				// Convert to string data type
		
		// Figure out the appropriate response
		if(command == "close")
		{
			output.addString("Arrivederci");
			run = false;							// This will terminate the while loop
		}
		else if(command == "read")						// Get the current joint positions
		{
			std::cout << "Retrieving encoder information.";
			while(!encoders->getEncoders(joint_position.data()))
			{
				yarp::os::Time::delay(0.01);
				std::cout << ".";
			}
			std::cout << std::endl;
			
			output.clear();
			for(int i = 0; i < n; i++)
			{
				output.addDouble(round(joint_position[i]));		// Return rounded values
			}
		}
		else if(command == "receive")						// Extend both arms to receive an object
		{
			output.addString("Grazie");
		}
		else if(command == "shake")						// Extend one arm to shake hands
		{
			output.addString("Piacere");
		}
		else if(command == "wave")						// Greet someone
		{
			output.addString("Ciao");
		}
		else
		{
			output.addString("Che cosa");
		}	
	
		port.reply(output);							// Send to the user
    	}
	
	return 0;									// No problems with main
}
