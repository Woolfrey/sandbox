#include <ArmController.h>
#include <yarp/os/RpcServer.h>

// Pre-defined positions
yarp::sig::Vector home({0.0, 10.0, 0.0, 16.0, 0.0, 0.0, 0.0});
yarp::sig::Vector receive({-50, 0, 0, 50, -60, 0, 0});
yarp::sig::Vector shake({-50, 20, 0, 50, 0, 0, 0});
yarp::sig::Vector wave({-30, 50, -30, 105, 30, 0, 0});

/********************* This is where the action happens *********************/
int main(int argc, char *argv[])
{
	// Create and configure the left arm
	std::string local = "/local/left";						// Local port names	
	std::string remote = "/icubSim/left_arm";					// Port names to connect with
	std::string name = "left_2.0"							// Identifier for object
	ArmController left_arm(local, remote, name);					// Create left arm object
	
	// Configure communication across yarp
	yarp::os::RpcServer port;							// Create a port for sending and receiving information
	port.open("/command");								// Open the port with the name /command
	yarp::os::Bottle input;							// Store information from user input
	yarp::os::Bottle output;							// Store information to send to user
	std::string command;								// Response message, command from user
	
	
	// Run the control
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
			left_arm.move_to_position(home);
			active = false;							// Shut down the while loop
		}
		
		// Return to a home position
		else if(command == "home")
		{
			output.addString("Casa");
			left_arm.move_to_position(home);
		}
		
		// Move the arm back and forth
		else if(command == "move")
		{
			output.addString("Moving...");
			left_arm.move_hand();
		}
		
		// Extend out both arms
		else if(command == "receive")
		{
			output.addString("Grazie");	
			left_arm.move_to_position(receive);
		}

		// Extend one arm to shake hands
		else if(command == "shake")
		{
			output.addString("Piacere");
			left_arm.move_to_position(shake);
		}

		// Wave one arm
		else if(command == "wave")
		{
			output.addString("Ciao");
			left_arm.move_to_position(wave);
		}
		
		// Move the hand up
		else if(command == "up")
		{
			output.addString("Up we go");
			left_arm.move_up();
		}
		else if(command == "down")
		{
			output.addString("Down we go");
			left_arm.move_down();
		}
		else if(command == "left")
		{
			output.addString("To the left");
			left_arm.move_left();
		}
			else if(command == "right")
		{
			output.addString("To the right");
			left_arm.move_right();
		}
		
		// Unknown command
		else
		{
			output.addString("Cosa");
		}
		
		port.reply(output);								// Send response to user
	}
	
	left_arm.close();
	
	left_arm.close();								// Close the device drivers
	

	return 0;									// No problems with main
}


