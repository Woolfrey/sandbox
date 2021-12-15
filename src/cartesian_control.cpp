/* NOTE: Temporarily commented out critical code */

#include <DualArmCtrl.h>
#include <yarp/os/RpcServer.h>

// Pre-defined positions
double d2r = M_PI/180;
yarp::sig::Vector home_torso({0.0, 0.0, 0.0});			
yarp::sig::Vector home_arm({-30*d2r, 30*d2r,  00*d2r, 45*d2r,  00*d2r,  00*d2r, 0});
yarp::sig::Vector receive( {-50*d2r, 10*d2r,  00*d2r, 40*d2r, -60*d2r,  20*d2r, 10*d2r});
yarp::sig::Vector shake(   {-50*d2r, 40*d2r,  65*d2r, 45*d2r, -70*d2r, -20*d2r, 0*d2r});
yarp::sig::Vector wave(    {-30*M_PI/180, 50*M_PI/180, -30*M_PI/180, 105*M_PI/180, 30*M_PI/180, 0, 0});

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
	
	
	controller.close();							// Close the device drivers
	
	return 0;								// No problems with main()
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
