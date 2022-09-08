    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     Interface with YARP to control the iCub in torque mode                     //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Humanoid.h>                                                                               // Custom robot control class
#include <JointConfigurationsiCub3.h>                                                               // Pre-programmed configurations for iCub3
#include <yarp/os/RpcServer.h>                                                                      // Ports for communicating with YARP

double shortTime = 2.0;
double longTime  = 5.0;

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
		return 1;                                                                           // Close with error
	}
	else
	{
		// Create the robot model
		std::string file = argv[1];                                                         // Get the urdf model path
		Humanoid robot(file);							            // Create object
		
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
			else if(command == "down")
			{
				robot.translate(iDynTree::Position(0.0, 0.0,-0.1),
						iDynTree::Position(0.0, 0.0,-0.1),
						longTime);
						
				output.addString("Down");
			}
			else if(command == "grasp")
			{		
				iDynTree::Transform leftPose(iDynTree::Rotation::RPY(0,0,0),
				                             iDynTree::Position(0.38,0.13,0.73));
				
				iDynTree::Transform rightPose(iDynTree::Rotation::RPY(0,0,0),
				                              iDynTree::Position(0.38,-0.13,0.73));
				                              
				robot.grasp_object(leftPose,rightPose);
				
				output.addString("Grasping");
			}			
			else if(command == "home")
			{
				robot.move_to_position(iDynTree::VectorDynSize(home), shortTime);
				output.addString("Casa");
			}
			else if(command == "in")
			{
				robot.translate(iDynTree::Position(0.0,-0.1,0.0),
						iDynTree::Position(0.0, 0.1,0.0),
						longTime);
						
				output.addString("In");
			}
			else if(command == "left")
			{
				robot.translate(iDynTree::Position(0.0,0.1,0.0),
						iDynTree::Position(0.0,0.1,0.0),
						longTime);
						
				output.addString("Sinistra");
			}
			else if(command == "out")
			{
				robot.translate(iDynTree::Position(0.0, 0.1,0.0),
						iDynTree::Position(0.0,-0.1,0.0),
						longTime);
						
				output.addString("Out");
			}
			else if(command == "ready")
			{
				iDynTree::Transform T1(iDynTree::Rotation::RPY(0,0,-0.2),
						       iDynTree::Position(0.3,0.23,0.75));
						       
				iDynTree::Transform T2(iDynTree::Rotation::RPY(0,0,0.2),
						       iDynTree::Position(0.3,-0.23,0.75));
						       
				robot.move_to_pose(T1,T2,longTime);
				
				output.addString("Pronto");
			}
			else if(command == "receive")
			{
				robot.move_to_position(iDynTree::VectorDynSize(receive), shortTime);
				
				output.addString("Grazie");
			}
			else if(command == "right")
			{
				robot.translate(iDynTree::Position(0.0,-0.1,0.0),
						iDynTree::Position(0.0,-0.1,0.0),
						longTime);
						
				output.addString("Destra");
			}
			else if(command == "shake")
			{
				robot.move_to_position(iDynTree::VectorDynSize(shake), shortTime);
				
				output.addString("Piacere");
			}
			else if(command == "stop")
			{
				robot.halt();                                                      // Stop control threads, maintain position
				output.addString("Fermata");
			}
			else if(command == "test")
			{
				iDynTree::Transform T(iDynTree::Rotation::RPY(0,0,0),
						      iDynTree::Position(0.3,0.2,0.8));
						      
				robot.move_to_pose(T, "left", longTime);
				
				output.addString("Testing");
			}
			else if(command == "up")
			{
				robot.translate(iDynTree::Position(0.0, 0.0, 0.1),
						iDynTree::Position(0.0, 0.0, 0.1),
						longTime);
						
				output.addString("Su");
			}
			else if(command == "wave")
			{
				robot.move_to_position(iDynTree::VectorDynSize(wave), shortTime);
				
				output.addString("Ciao");
			}
			else
			{
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
