#include <Haiku.h>										// A simple poem in the form of a haiku
#include <Humanoid.h>										// Custom robot control class
#include <yarp/os/RpcServer.h>									// Ports for communicating with YARP

/////////////////////////////////////////////////////////////////////////////////////////////////////
//				Pre-defined joint configurations				  //
////////////////////////////////////////////////////////////////////////////////////////////////////
std::vector<double> home({	 00.00,  00.00,  00.00,						// Torso
				-30.00,  30.00,  00.00,  45.00,  00.00,  00.00,  00.00,		// Left arm
				-30.00,  30.00,  00.00,  45.00,  00.00,  00.00,  00.00});	// Right arm

std::vector<double> receive({	 00.00,  00.00,  00.00,						// Torso
				-50.00,  10.00,  00.00,  40.00, -60.00,  20.00,  10.00,		// Left arm
				-50.00,  10.00,  00.00,  40.00, -60.00,  20.00,  00.00});	// Right arm
				
std::vector<double> shake({	 00.00,  00.00,  00.00,						// Torso
				-30.00,  30.00,  00.00,  45.00,  00.00,  00.00,  00.00,		// Left arm
				-50.00,  40.00,  65.00,  45.00, -70.00, -20.00,  00.00});	// Right arm

std::vector<double> wave({	 00.00,  00.00,  00.00,						// Torso
				-30.00,  50.00, -30.00, 105.00,  30.00,  00.00,  00.00,		// Left arm
				-30.00,  30.00,  00.00,  45.00,  00.00,  00.00,  00.00});	// Right arm
					
/////////////////////////////////////////////////////////////////////////////////////////////////////
//						MAIN						  //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	// Convert from degrees to radians
	for(int i = 0; i < home.size(); i++)
	{
		home[i] 	*= M_PI/180;
		receive[i] 	*= M_PI/180;
		shake[i] 	*= M_PI/180;
		wave[i] 	*= M_PI/180;
	}

	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 2)									
	{
		std::cerr << "[ERROR] [IMPEDANCECONTROL] Path to urdf model required."
			<< " Usage: './impedance_control /path/to/model.urdf'" << std::endl;
		return 1;								// Close with error
	}
	else
	{
		// Create the robot model
		std::string file = argv[1];						// Get the urdf model path
		Humanoid robot(file);							// Create object
		
		// Configure communication across the yarp network
		yarp::os::Network yarp;							// First connect to the network
		yarp::os::RpcServer port;						// Create a port for sending / receiving info
		port.open("/command");							// Open the port with the name '/command'
		yarp::os::Bottle input;							// Store information from the user input
		yarp::os::Bottle output;						// Store information to send to the user
		std::string command;							// Response message, command from user
		
		bool active = true;
		while(active)
		{
			port.read(input, true);						// Read from the '/command' port
			command = input.toString();					// Convert to a string
			
			if(command == "close")
			{
				robot.stop();						// Stop any control threads
				output.addString("Arrivederci");
				active = false;						// This will break the 'while' loop
			}
			else if(command == "home")
			{
				robot.move_to_position(iDynTree::VectorDynSize(home));
				output.addString("Casa");
			}
			else if(command == "receive")
			{
//				controller.move_to_position(receive);
				output.addString("Grazie");
			}
			else if(command == "stop")
			{
				robot.halt();						// Stop any control threads, maintain current position
				output.addString("Fermata");
			}
			else if(command == "wave")
			{
//				controller.move_to_position(const iDynTree::VectorDynSize &position);
				output.addString("Ciao");
			}
			else
			{
				output.addString("Cosa");
			}
			
			port.reply(output);						// Send the reply message over the network
			output.clear();							// Clear the output for the next loop
		}	

		robot.close();								// Close the communication with the robot
		std::cout << "[INFO] [IMPEDANCECONTROL] All done." << std::endl;
		
		return 0;								// No problems with main
	}
}
