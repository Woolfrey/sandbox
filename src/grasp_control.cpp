    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     Interface with YARP to control the iCub in torque mode                     //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <BiManualGrasp.h>                                                                          // Custom robot control class
#include <JointConfigurationsiCub3.h>                                                               // Pre-programmed configurations for iCub3
#include <yarp/os/RpcServer.h>                                                                      // Ports for communicating with YARP

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                                MAIN                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	// Default for argc is 1, but I don't know why ¯\_(ツ)_/¯
	if(argc != 2)									
	{
		std::cerr << "[ERROR] [IMPEDANCECONTROL] Path to urdf model required."
			  << " Usage: './grasp_control /path/to/model.urdf'" << std::endl;
		return 1;                                                                           // Close with error
	}
	else
	{
		// Create the robot model
		std::string file = argv[1];                                                         // Get the urdf model path
		BiManualGrasp robot(file);							    // Create object
		
		// Configure communication across the yarp network
		yarp::os::Network yarp;                                                             // First connect to the network
		yarp::os::RpcServer port;                                                           // Create a port for sending / receiving info
		port.open("/command");                                                              // Open the port with the name '/command'
		yarp::os::Bottle input;                                                             // Store information from the user input
		yarp::os::Bottle output;                                                            // Store information to send to the user
		std::string command;                                                                // Response message, command from user
		
		bool active = true;
		while(active)
		{
			port.read(input, true);                                                     // Read from the '/command' port
			command = input.toString();                                                 // Convert to a string
			
			if(command == "close")
			{
				robot.stop();                                                       // Stop any control threads
				output.addString("Arrivederci");
				active = false;                                                     // This will break the 'while' loop
			}
			else if(command == "home")
			{
				robot.move_to_position(iDynTree::VectorDynSize(home), 2.0);
				output.addString("Casa");
			}
			else if(command == "stop")
			{
				robot.halt();                                                      // Stop control threads, maintain position
				output.addString("Fermata");
			}
			else if(command == "grasp")
			{
				output.addString("Grasping");
				
				iDynTree::Transform object(iDynTree::Rotation::RPY(0,0,0),          // Pose of object
				                           iDynTree::Position(0.4, 0, 0.68));
				                           
				iDynTree::Transform left(iDynTree::Rotation::RPY(0,0,0),            // Left hand grasp pose
				                         iDynTree::Position(0.3, 0.15, 0.70));
				
				iDynTree::Transform right(iDynTree::Rotation::RPY(0,0,0),           // Right hand grasp pose
                                                          iDynTree::Position(0.3, -0.15, 0.70));
                                                      
				robot.grasp_object(left, right, object);
			}
			
			port.reply(output);                                                        // Send the reply message over the network
			output.clear();                                                            // Clear the output for the next loop
		}	

		robot.close();                                                                     // Close the communication with the robot
		std::cout << "[INFO] [IMPEDANCECONTROL] All done." << std::endl;
		
		return 0;                                                                          // No problems with main
	}
}
