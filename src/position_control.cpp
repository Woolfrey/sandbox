// #include <cstdio>							
#include <cmath>							// round() function
#include <iostream>							// Print to console
#include <string>							// std::string object
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>						// yarp::os::Bottle for simple communications
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>						// Non so?
#include <yarp/os/Property.h>						// Don't know exactly what this does
#include <yarp/os/RpcServer.h>					// Allows communication between ports
#include <yarp/sig/Vector.h>						// Linear algebra for signal processing

// Global variables
int n;
yarp::sig::Vector joint_position;					// Joint position information
yarp::sig::Vector target;

bool configureArmControl(yarp::dev::IEncoders &enc, yarp::dev::IPositionControl &controller)
{

	return true;
}

void getEncoderValues(yarp::dev::IEncoders &enc, yarp::sig::Vector &storage)
{
	//yarp::os::LogStream yInfo() << "Retrieving encoder information...";
	while(!enc.getEncoders(storage.data()))
	{
		yarp::os::Time::delay(0.01);
	}
	//yarp::os::LogStream yInfo() << "Done.";
}

void home(yarp::sig::Vector &setpoint, yarp::dev::IPositionControl &control)
{
	setpoint[0] = 00.0;
	setpoint[1] = 20.0;
	setpoint[2] = 00.0;
	setpoint[3] = 00.0;
	setpoint[4] = 00.0;
	control.positionMove(setpoint.data());			// Move to home position	
}

void receive(yarp::sig::Vector &setpoint, yarp::dev::IPositionControl &control)
{
	setpoint[0] = -50.0;
	setpoint[1] =  00.0;
	setpoint[2] =  00.0;
	setpoint[3] =  50.0;
	setpoint[4] = -90.0;
	control.positionMove(setpoint.data());			// Move to home position	
}

void shake(yarp::sig::Vector &setpoint, yarp::dev::IPositionControl &control)
{
	setpoint[0] = -50.0;
	setpoint[1] =  20.0;
	setpoint[2] =  00.0;
	setpoint[3] =  50.0;
	setpoint[4] =  00.0;
	control.positionMove(setpoint.data());			// Move to home position	
}

void wave(yarp::sig::Vector &setpoint, yarp::dev::IPositionControl &control)
{
	setpoint[0] =  -30.0;
	setpoint[1] =   50.0;
	setpoint[2] =  -60.0;
	setpoint[3] =  100.0;
	setpoint[4] =   30.0;
	control.positionMove(setpoint.data());			// Move to home position
}

int main(int argc, char *argv[])
{
	yarp::os::Network yarp;					// Set up YARP
	
	// Configure communication
	yarp::os::RpcServer port;					// Create a port for sending and receiving information
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
 	if(!robotDevice.view(controller));// yarp::os::Network yError() << "Problems acquiring controller interface.";
 	controller->getAxes(&n);					// Get the number of motors that can be controlled
 	target.resize(n);						// Resize vector for target joint positiosn
	yarp::sig::Vector temp;					// Temporary storage vector
	temp.resize(n);						// Resize temp storage vector
 	for(int i = 0; i < n; i++)					// Iterate through each joint...
 	{
 		controller->setRefSpeed(i,30.0);			// Set reference speed (deg/s) for position controller
 		temp[i] = 80;						// Acceleration (deg/s^2)
 	}		
 	controller->setRefAccelerations(temp.data());			// Set reference accelerations (deg/s^2) (WHY LIKE THIS?)
 	
 	
 	// Configure encoder information
 	yarp::dev::IEncoders *encoders;				// Needs a pointer for some reason?
 	if(!robotDevice.view(encoders));//yarp::os::LogStream yError() << "Problems acquiring encoder interface.";
 	joint_position.resize(n);					// Joint position information to be received from the encoders
 	
 	
 	// Get and set initial joint values
 	getEncoderValues(*encoders, joint_position);
 	target = joint_position;
 	home(target, *controller);
 	
 	
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
			home(target,*controller);
			output.addString("Arrivederci");
			run = false;							// This will terminate the while loop
		}
		
		
		else if(command == "home")
		{
			home(target, *controller);					// Move to home position
			output.addString("A casa");
		}
		
		// Get the current joint positions
		else if(command == "read")
		{
			getEncoderValues(*encoders,joint_position);
			
			for(int i = 0; i < n; i++)
			{
				output.addDouble(joint_position[i]);		// Return rounded values
			}
		}
		
		
		// Extend both arms
		else if(command == "receive")
		{
			receive(target, *controller);
			output.addString("Grazie");
		}
		
		
		// Extend one arm
		else if(command == "shake")
		{
			shake(target, *controller);
			output.addString("Piacere");
		}
		
		
		// Wave one arm
		else if(command == "wave")
		{
			wave(target, *controller);
			output.addString("Ciao");
		}
		
		
		// Unknown command
		else
		{
			output.addString("Che cosa");
		}	
	
		port.reply(output);							// Send to the user
    	}
	
	robotDevice.close();								// Close connection with the robot
	return 0;									// No problems with main
}
