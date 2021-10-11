#include <ArmController.h>
#include <yarp/os/RpcServer.h>


/********************* This is where the action happens *********************/
int main(int argc, char *argv[])
{

	// Create and configure the left arm
	std::string local = "/local/left";
	std::string remote = "/icubSim/left_arm";
	std::string name = "left";
	ArmController left_arm(local, remote, name);
	
	// Create and configure the right arm
	local = "/local/right";
	remote = "/icubSim/right_arm";
	name = "right";
	ArmController right_arm(local, remote, name);
		
	// Move to a home position
	yarp::sig::Vector home(16);
	home.setSubvector(0, yarp::sig::Vector({0.0, 10.0, 0.0, 16.0, 0.0, 0.0, 0.0}));
	left_arm.move_to_position(home);
	right_arm.move_to_position(home);
	
	// Configure communication across yarp
	yarp::os::RpcServer port;								// Create a port for sending and receiving information
	port.open("/command");									// Open the port with the name /command
	yarp::os::Bottle input;								// Store information from user input
	yarp::os::Bottle output;								// Store information to send to user
	std::string command;									// Response message, command from user
	
	// Run the command interface / control loop
	bool active = true;
	
	while(active)
	{
		output.clear();								// Clear any previous
		
		bool control = false;								// Don't move the robot
		
		port.read(input, true);							// Get any commands over the network
		command = input.toString();							// Convert to a string
		
		// Shut down the robot
		if(command == "close")
		{
			output.addString("Arrivederci");
			left_arm.move_to_position(home);
			right_arm.move_to_position(home);
			active = false;							// Shut down the while loop
		}
		
		// Return to a home position
		else if(command == "home")
		{
			output.addString("Casa");
			left_arm.move_to_position(home);
			right_arm.move_to_position(home);	
		}
		
		// Extend out both arms
		else if(command == "receive")
		{
			output.addString("Grazie");	
			yarp::sig::Vector setpoint(16);
			setpoint.setSubvector(0, yarp::sig::Vector({-50, 0, 0, 50, -60, 0, 0}));
			left_arm.move_to_position(setpoint);
			right_arm.move_to_position(setpoint);			
		}

		// Extend one arm to shake hands
		else if(command == "shake")
		{
			output.addString("Piacere");
			yarp::sig::Vector setpoint(16);
			setpoint.setSubvector(0, yarp::sig::Vector({-50, 20, 0, 50, 0, 0, 0}));
			left_arm.move_to_position(home);
			right_arm.move_to_position(setpoint);	
		}
		
		// Move one hand up
		else if(command == "up")
		{
			output.addString("Up we go!");
			left_arm.move_up();
		}
		else if(command == "down")
		{
			output.addString("Down we go!");
			left_arm.move_down();
		}
		else if(command == "left")
		{
			output.addString("To the left.");
			left_arm.move_left();
		}
		else if(command == "right")
		{
			output.addString("To the right.");
			left_arm.move_right();
		}
		
		else if(command == "jacobian")
		{
			output.addString("Check the other terminal.");
			left_arm.print_jacobian();
		}

		// Wave one arm
		else if(command == "wave")
		{
			output.addString("Ciao");
			yarp::sig::Vector setpoint(16);
			setpoint.setSubvector(0, yarp::sig::Vector({-30, 50, -30, 105, 30, 0, 0}));
			left_arm.move_to_position(home);
			right_arm.move_to_position(setpoint);
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
	
	
	return 0;									// No problems with main
}


