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


void getEncoderValues(yarp::dev::IEncoders &enc, yarp::sig::Vector &storage)
{
	while(!enc.getEncoders(storage.data()))
	{
		yarp::os::Time::delay(0.01);
	}
}

bool configureDriver(yarp::dev::PolyDriver &driver, std::string &local, std::string &remote)
{
	yarp::os::Property options;
	options.put("device", "remote_controlboard");
	options.put("local", local);				// Local port names
	options.put("remote", remote);			// Where we want to connect to
	
	driver.open(options);
	
	if(!driver.isValid())					
	{
		// yError() << "Message.";
		
		std::printf("Device not available. Here are the known devices:\n");		// Inform user
		std::printf("%s", yarp::dev::Drivers::factory().toString().c_str());	
		return 0;									// Shut down
	}			
	else
	{
		// yInfo() << "Message";
		
		return 1;
	}
}

bool configureControl(yarp::dev::PolyDriver &driver, yarp::dev::IPositionControl* &controller, yarp::sig::Vector &target)
{
	if(!driver.view(controller))
	{
		// yError() << "Message.";
		
		return 0;						// Unsuccessful
	}
	else
	{
		int n;
		controller->getAxes(&n);				// Get the number of joints
		
		target.resize(n);					// Resize the target vector accordingly
		
		yarp::sig::Vector temp;				// Temporary storage vector
		temp.resize(n);
		
		for(int i = 0; i < n; i++)
		{
			controller->setRefSpeed(i,50.0);		// Max speed (deg/s) for position controller
			temp[i] = 100.0;				// (deg/s^2);
		}
		controller->setRefAccelerations(temp.data());		// WHY LIKE THIS?
		
		// yInfo() << "Message.";
		
		return 1;						// Success	
	}
}

bool configureEncoder(yarp::dev::PolyDriver &driver, yarp::dev::IEncoders* &encoder, yarp::sig::Vector &position)
{
	if(!driver.view(encoder))
	{
		// yError() << "Message";
		
		return 0;						// Unsuccessful
	}
	else
	{
		int n;
		encoder->getAxes(&n);					// Get the number of joints
		position.resize(n);					// Resize position vector accordingly
		getEncoderValues(*encoder, position);			// Assign initial values
		// yInfo() << "Message";
		return 1;						// Success
	}
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
	setpoint[4] = -60.0;
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

	/******************************** This bock of code works! *************************************************/
	/* Configure right arm
	yarp::os::Property options;
	options.put("device", "remote_controlboard");
	options.put("local", "/client/right");				// Local port names
	options.put("remote", "/icubSim/right_arm");			// Where we want to connect to
	yarp::dev::PolyDriver driver;
	driver.open(options);
	if(!driver.isValid())					
	{
		// yError() << "Message.";
		
		std::printf("Device not available. Here are the known devices:\n");		// Inform user
		std::printf("%s", yarp::dev::Drivers::factory().toString().c_str());	
		return 0;									// Shut down
	}			
	
	// Configure left arm driver
	options.put("device", "remote_controlboard");
	options.put("local", "/client/left");				// Local port names
	options.put("remote", "/icubSim/left_arm");			// Where we want to connect to
	yarp::dev::PolyDriver driver2;
	driver2.open(options);
	if(!driver2.isValid())					
	{
		// yError() << "Message.";
		
		std::printf("Device not available. Here are the known devices:\n");		// Inform user
		std::printf("%s", yarp::dev::Drivers::factory().toString().c_str());	
		return 0;									// Shut down
	}			
	else
	{
		// yInfo() << "Message";
		
		return 1;
	} */
	/************************************************************************************************************/
	
	
	// Configure the left arm
	std::string local = "/local/left";
	std::string remote = "/icubSim/left_arm";
	std::string name = "left_arm";			
	yarp::dev::PolyDriver left_driver;				// Create device driver object
	if(!configureDriver(left_driver, local, remote)) return 1;	// Configure and connect to iCub_SIM

	yarp::dev::IPositionControl* left_control;
	yarp::sig::Vector left_target;
	if(!configureControl(left_driver, left_control, left_target)) return 1;
	
	yarp::dev::IEncoders* left_encoder;
	yarp::sig::Vector left_position;
	if(!configureEncoder(left_driver, left_encoder, left_position)) return 1;
	
	// Configure the right arm
	local = "/local/right";
	remote = "/icubSim/right_arm";
	yarp::dev::PolyDriver right_driver;
	if(!configureDriver(right_driver, local, remote)) return 1;
	
	yarp::dev::IPositionControl* right_control;
	yarp::sig::Vector right_target;
	if(!configureControl(right_driver, right_control, right_target)) return 1;
	
	yarp::dev::IEncoders* right_encoder;
	yarp::sig::Vector right_position;
	if(!configureEncoder(right_driver, right_encoder, right_position)) return 1;	
	
	// Move to starting position
	left_target = left_position;					// Set initial values read from encoders
	right_target = right_position;
	home(left_target, *left_control);				// Move to pre-programmed joint configuration
	home(right_target, *right_control);
	
	// Configure communication
	yarp::os::RpcServer port;					// Create a port for sending and receiving information
	port.open("/command");						// Open the port with the name /command
	yarp::os::Bottle input;					// Store information from user input
	yarp::os::Bottle output;					// Store information to send to user
	std::string command;						// Response message, command from user
	
	// Receive commands and respond appropriately	
	bool run = true;
	while(run)
	{
		output.clear();					// Remove any previous information
		
		port.read(input, true);				// Get input commands
		command = input.toString();				// Convert to string data type
		
		// Shut down the executable
		if(command == "close")
		{
			home(left_target, *left_control);
			home(right_target, *right_control);
			output.addString("Arrivederci");
			run = false;
		}
		
		// Move arms to the starting position
		else if(command == "home")
		{
			home(left_target, *left_control);
			home(right_target, *right_control);
			output.addString("aCasa");
		}
		
		// Extend arms to receive an object		
		else if(command == "receive")
		{
			receive(left_target, *left_control);
			receive(right_target, *right_control);
			output.addString("Grazie");
		}
		
		
		// Extend one arm 
		else if(command == "shake")
		{
			home(left_target, *left_control);
			shake(right_target, *right_control);
			output.addString("Piacere");
		}
		
		
		// Wave one arm
		else if(command == "wave")
		{
			home(left_target, *left_control);
			wave(right_target, *right_control);
			output.addString("Ciao");
		}
		
		
		// Unknown command
		else
		{
			output.addString("Che cosa");
		}	
	
		port.reply(output);							// Send to the user

    	}
    	
	return 0;									// No problems with main
}
