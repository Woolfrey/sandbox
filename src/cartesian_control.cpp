/* NOTE: Temporarily commented out critical code */

#include <DualArmController.h>
#include <yarp/os/RpcServer.h>

// Pre-defined positions
yarp::sig::Vector home({-30.0, 30.0,  00.0,  45.0, 00.0, 00.0, 00.0});		// Starting pose
yarp::sig::Vector receive({-50.0, 00.0, 00.0, 50.0, -60.0, 00.0, 00.0});		// Extend arm out, palm up
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
		
		// Shut down the robot
		if(command == "close")
		{
			output.addString("Arrivederci");
			active = false;						// Shut down the while loop
		}
		
		// Move both hands up
		if(command == "freeze")
		{
			controller.move_to_position(wave, wave);
			output.addString("Gasp!");
		}
		
		// Move hands to a grasp position
		if(command == "grasp")
		{
			controller.move_to_position(shake, shake);
			output.addString("Pronto");
		}
		
		// Move the arms back to the starting position
		if(command == "home")
		{
			controller.move_to_position(home, home);
			output.addString("Casa");
		}
		
		// Move the hands inwards
		if(command == "in")
		{
			controller.move_lateral(0.1);
			output.addString("Closer");
		}
		if(command == "out")
		{
			controller.move_lateral(-0.1);
			output.addString("Further");
		}
		
		// Get the current joint positions
		if(command == "read")
		{
			yarp::sig::Vector q = controller.get_positions("left");	// Temporary storage location
			for(int i = 0; i < q.size(); i++) output.addFloat64(q[i]); // Return rounded values
		}
		
		// Extend one arm to shake hands
		if(command == "shake")
		{
			controller.move_to_position(home, shake);
			output.addString("Piacere");
		}
		
		// Wave one hand
		if(command == "wave")
		{
			controller.move_to_position(home, wave);			// Raise 1 hand to wave
			output.addString("Ciao");
		}
		
		port.reply(output);
	}
		
	controller.close();
		
	return 0;									// No problems with main
}
