/* NOTE: Temporarily commented out critical code */

#include <DualArmCtrl.h>
#include <yarp/os/RpcServer.h>

// Pre-defined positions
yarp::sig::Vector home_torso({0.0, 0.0, 0.0});			
yarp::sig::Vector home_arm({-30, 30,  00,  45,  00,  00,  0});
yarp::sig::Vector receive( {-50, 10,  00,  40, -60,  20, 10});
yarp::sig::Vector shake(   {-50, 40,  65,  45, -70, -20,  0});
yarp::sig::Vector wave(    {-30, 50, -30, 105,  30,   0,  0});		


// Forward declaration - for readability
std::string process_command(const std::string &input, DualArmCtrl &robot);

/********************* This is where the action happens *********************/
int main(int argc, char *argv[])
{
	yarp::os::Network yarp;							// Connect to the yarp connectwork
	
	DualArmCtrl controller(200);						// Create dual arm controller object

	// Configure communication across yarp
	yarp::os::RpcServer port;						// Create a port for sending and receiving information
	port.open("/command");							// Open the port with the name /command
	yarp::os::Bottle input;							// Store information from user input
	yarp::os::Bottle output;						// Store information to send to user
	std::string command;							// Response message, command from user
	
	// Run the control loop
	bool controlActive = true;
	
	while(controlActive)
	{
		output.clear();
		port.read(input, true);
		command = input.toString();
		
		if(command == "close") 						// Shut down the robot
		{
			output.addString("Arrivederci");
			controlActive = false;
		}
		else
		{
			output.addString(process_command(command, controller)); // This is just to make the code a bit more readable
		}
		
		port.reply(output);						// Respond to the user
	}
	
	controller.close();							// Close the device drivers
	
	return 0;								// No problems with main()
}

std::string process_command(const std::string &input, DualArmCtrl &robot)
{
	int space;
	std::string response;
	yarp::sig::Vector torso(3), leftArm(7), rightArm(7);
	yarp::sig::Matrix leftHand(4,4), rightHand(4,4);

	// Joint Control Options
	
	
	// Cartesian Control Options
	if(input == "up")
	{
		space = 2;
		response = "Su";
	}
	else if(input == "down")
	{
		space = 2;
		response = "Giu`";
	}
	else if(input == "left")
	{
		space = 2;
		response = "Sinistra";
	}
	else if(input == "right")
	{
		space = 2;
		response = "Destra";
	}
	else if(input == "straighten")
	{
		space = 2;
		response = "Ricevuto";
	}
	else if(input == "manipulability")
	{
		space = 2;
		response = "Ricevuto";
		robot.set_redundant_task(1, 5.0);
		leftHand = robot.get_hand_pose("left");
		rightHand = robot.get_hand_pose("right");
	}
	else if(input == "stiffness")
	{
		space = 2;
		response = "Ricevuto";
		robot.set_redundant_task(2,-3.0);
		leftHand = robot.get_hand_pose("left");
		rightHand = robot.get_hand_pose("right");
	}
	
	// Default option
	else
	{
		space = 0;
		response = "Che cosa?";
	}
	
	switch(space)
	{
		case 1:
		{
			break;
		}
		case 2:
		{
			robot.move_to_pose(leftHand, rightHand);
			break;
		}
		default:
		{
			break;
		}
	}
	
	return response;
}

/*	
	while(controlActive)
	{
		output.clear();							// Clear any previous messages
		
		port.read(input, true);						// Read messages over the network
		command = input.toString();					// Convert to a string
		
		/******************** Interfacing ********************
		if(command == "close") // Shut down the robot
		{
			output.addString("Arrivederci");
			controlActive = false;
		}
		else if(command == "print joints")
		{
			output.addString("Ecco qui");
			controller.update_state();
		}
		else if(command == "stop")
		{
			output.addString("Fermata");
			controller.stop();
		}
		else if(command == "left pose")
		{
			controller.print_pose("left");
		}
		else if(command == "right pose")
		{
			controller.print_pose("right");
		}
		
		/******************** Joint-level control ********************
		
		else if(command == "freeze") // Put both hands up
		{
			controller.move_to_position(wave, wave, home_torso);
			output.addString("No");
		}
		else if(command == "grasp") // Move both hands to a grasp position
		{
			controller.move_to_position(shake, shake, home_torso);
			output.addString("Pronto");
		}
		else if(command == "home") // Move the arms back to the start
		{
			controller.move_to_position(home_arm, home_arm, home_torso);
			output.addString("Casa");
		}
		else if(command == "shake") // Extend one arm
		{
			controller.move_to_position(home_arm, shake, home_torso);
			output.addString("Piacere");
		}
		else if(command == "receive")
		{
			controller.move_to_position(receive, receive, home_torso);
			output.addString("Grazie");
		}
		
		else if(command == "wave") // Wave one hand
		{
			controller.move_to_position(home_arm, wave, home_torso);	// Raise 1 hand to wave
			output.addString("Ciao");
		}
		
		/******************** Cartesian-level control ********************
		else if(command == "left")
		{
			controller.move_lateral(0.1);
			output.addString("Sinistra");
		}
		else if(command == "right")
		{
			controller.move_lateral(-0.1);
			output.addString("Destra");
		}
		else if(command == "up")
		{
			controller.move_vertical(0.1);
			output.addString("Su");
		}
		else if(command == "down")
		{
			controller.move_vertical(-0.1);
			output.addString("Giu");
		}
		else if(command == "in")
		{	controller.move_in_out(0.05);
			output.addString("Vicino");
		}
		else if(command == "out")
		{
			controller.move_in_out(-0.05);
			output.addString("Lontano");
		}
		else if(command == "forward")
		{
			controller.move_horizontal(0.1);
		}
		else if(command == "backward")
		{
			controller.move_horizontal(-0.1);
		}
		/****************************************************************
		
		else
		{
			output.addString("Che cosa?");
		}
		
		port.reply(output);						// Send a message to the user
	}
	controller.close();
	return 0;
}*/
