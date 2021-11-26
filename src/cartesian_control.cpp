/* NOTE: Temporarily commented out critical code */

#include <DualArmController.h>
#include <yarp/os/RpcServer.h>

// Pre-defined positions
yarp::sig::Vector home_arm({-30.0, 30.0,  00.0,  45.0, 00.0, 00.0, 00.0});	// Starting pose
yarp::sig::Vector home_torso({0.0, 0.0, 0.0});					
yarp::sig::Vector receive({-50.0, 00.0, 00.0, 50.0, -60.0, 00.0, 00.0});	// Extend arm out, palm up
yarp::sig::Vector shake({-50.0, 20.0, 00.0, 50.0, 00.0, 00.0, 00.0});		// Extend arm out, palm sideways
yarp::sig::Vector wave({-30.0, 50.0, -30.0, 105.0, 30.0, 00.0, 00.0});		// Raise hand up

/********************* This is where the action happens *********************/
int main(int argc, char *argv[])
{
	yarp::os::Network yarp;							// Connect to the yarp connectwork
	
	DualArmController controller;						// Create dual arm controller object

	// Configure communication across yarp
	yarp::os::RpcServer port;						// Create a port for sending and receiving information
	port.open("/command");							// Open the port with the name /command
	yarp::os::Bottle input;							// Store information from user input
	yarp::os::Bottle output;						// Store information to send to user
	std::string command;							// Response message, command from user
		
	// Run the control loop
	bool active = true;
	
	while(active)
	{
		output.clear();							// Clear any previous
	
		port.read(input, true);						// Get any commands over the network
		command = input.toString();					// Convert to a string
		
		/******************** Interfacing ********************/

		if(command == "close") // Shut down the robot
		{
			output.addString("Arrivederci");
			active = false;							// Shut down the while loop
		}
		else if(command == "read") // Read the current joint positions
		{
			yarp::sig::Vector q = controller.get_positions("torso");	// Temporary storage location
			for(int i = 0; i < q.size(); i++) output.addFloat64(q[i]); 	// Return rounded values
		}
		else if(command == "test")
		{
			controller.blah();
			output.addString("Correta?");
		}
		
		/******************** Joint-level control ********************/
		
		else if(command == "freeze") // Put both hands up
		{
			controller.move_to_position(wave, wave, home_torso);
			output.addString("Fermata");
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
		else if(command == "wave") // Wave one hand
		{
			controller.move_to_position(home_arm, wave, home_torso);	// Raise 1 hand to wave
			output.addString("Ciao");
		}
		
		/******************** Cartesian-level control ********************/
		
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
		
		/*******************************************************************/
		
		else
		{
			output.addString("Che cosa?");
		}
				
		port.reply(output);
	}
		
	controller.close();
		
	return 0;									// No problems with main
}
