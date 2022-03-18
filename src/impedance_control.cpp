    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     Interface with YARP to control the iCub in torque mode                     //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Haiku.h>                                                                                 // A simple poem in the form of a haiku
#include <Humanoid.h>                                                                              // Custom robot control class
#include <JointConfigurationsiCub3.h>                                                              // Pre-programmed configurations for iCub3
#include <yarp/os/RpcServer.h>                                                                     // Ports for communicating with YARP


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                                MAIN                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 2)									
	{
		std::cerr << "[ERROR] [IMPEDANCECONTROL] Path to urdf model required."
			  << " Usage: './impedance_control /path/to/model.urdf'" << std::endl;
		return 1;                                                                          // Close with error
	}
	else
	{
		// Create the robot model
		std::string file = argv[1];                                                        // Get the urdf model path
		Humanoid robot(file);							           // Create object
		
		// Configure communication across the yarp network
		yarp::os::Network yarp;                                                            // First connect to the network
		yarp::os::RpcServer port;                                                          // Create a port for sending / receiving info
		port.open("/command");                                                             // Open the port with the name '/command'
		yarp::os::Bottle input;                                                            // Store information from the user input
		yarp::os::Bottle output;                                                           // Store information to send to the user
		std::string command;                                                               // Response message, command from user
		
		bool active = true;
		while(active)
		{
			port.read(input, true);                                                    // Read from the '/command' port
			command = input.toString();                                                // Convert to a string
			
			if(command == "close")
			{
				robot.stop();                                                      // Stop any control threads
				output.addString("Arrivederci");
				active = false;                                                    // This will break the 'while' loop
			}
			else if(command == "home")
			{
				robot.move_to_position(iDynTree::VectorDynSize(home));
				output.addString("Casa");
			}
			else if(command == "receive")
			{
				robot.move_to_position(iDynTree::VectorDynSize(receive));
				output.addString("Grazie");
			}
			else if(command == "shake")
			{
				robot.move_to_position(iDynTree::VectorDynSize(shake));
				output.addString("Piacere");
			}
			else if(command == "stop")
			{
				robot.halt();                                                      // Stop any control threads, maintain current position
				output.addString("Fermata");
			}
			else if(command == "test")
			{
				iDynTree::Transform T = robot.get_hand_pose("left");               // Get the left hand pose
				iDynTree::Position p = T.getPosition();                            // Get the position
				p[1] -= 0.2;                                                       // Move to the right
				T.setPosition(p);                                                  // Override
				if(robot.move_to_pose(T,"left")) output.addString("Testing");
				else                             output.addString("Problema");
			}
			else if(command == "wave")
			{
				robot.move_to_position(iDynTree::VectorDynSize(wave));
				output.addString("Ciao");
			}
			else
			{
				// List of waypoints to move through
				std::vector<iDynTree::VectorDynSize> positions;
				positions.push_back(iDynTree::VectorDynSize(hmmm));
				positions.push_back(iDynTree::VectorDynSize(neutral));
				
				// List of times to reach each waypoint
				std::vector<double> times;
				times.push_back(2);
				times.push_back(4);
				
				robot.move_to_positions(positions, times);
				output.addString("Cosa");
			}
			
			port.reply(output);                                                        // Send the reply message over the network
			output.clear();                                                            // Clear the output for the next loop
		}	

		robot.close();                                                                     // Close the communication with the robot
		std::cout << "[INFO] [IMPEDANCECONTROL] All done." << std::endl;
		
		return 0;                                                                          // No problems with main
	}
}
