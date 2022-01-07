#include <DualArmCtrl.h>
#include <yarp/os/RpcServer.h>

// Pre-defined positions
yarp::sig::Matrix RL = yarp::math::rpy2dcm(yarp::sig::Vector({-1.15*M_PI/2, 0.0,-0.0}));	// Keeps the palms vertical
yarp::sig::Matrix RR = yarp::math::rpy2dcm(yarp::sig::Vector({-0.85*M_PI/2, 0.0, 0.0}));	// Keep the palms vertical
yarp::sig::Vector armHome({-30, 30,  00,  45,  00,  00,  0});
yarp::sig::Vector graspPos({0.4, 0.19, 0.63});
yarp::sig::Vector receive( {-50, 10,  00,  40, -60,  20, 10});
yarp::sig::Vector shake(   {-50, 40,  65,  45, -70, -20,  0});
yarp::sig::Vector torsoHome({0.0, 0.0, 0.0});
yarp::sig::Vector wave(    {-30, 50, -30, 105,  30,   0,  0});

/********************* This is where the action happens *********************/
int main(int argc, char *argv[])
{
	// Convert vectors from deg to rad because YARP is annoying
	armHome *= M_PI/180;
	receive *= M_PI/180;
	shake *= M_PI/180;
	torsoHome *= M_PI/180;
	wave *= M_PI/180;
	
	yarp::os::Network yarp;							// Connect to the yarp connectwork
	
	DualArmCtrl controller(100);						// Create dual arm controller object

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
		
		if(command == "back")
		{
			output.addString("Indietro");
			controller.translate(	yarp::sig::Vector({-0.1, 0.0, 0.0}),
						yarp::sig::Vector({-0.1, 0.0, 0.0}));
		}
		else if(command == "base pose")					// Print the base pose
		{
			output.addString("Check the other terminal!");
			controller.print_pose("base");
		}		
		else if(command == "close") 					// Shut down the robot
		{
			output.addString("Arrivederci");
			controller.stop();					// Stop any control threads
			controlActive = false;					// Stop the while loop
		}
		else if(command == "down")					// Move both hands down
		{
			output.addString("Giu");
			controller.translate(	yarp::sig::Vector({ 0.0, 0.0,-0.1}),
						yarp::sig::Vector({ 0.0, 0.0,-0.1}));
		}
		else if(command == "forward")					// Move both hands forward
		{
			output.addString("Avanti");
			controller.translate(	yarp::sig::Vector({ 0.1, 0.0, 0.0}),
						yarp::sig::Vector({ 0.1, 0.0, 0.0}));
		}
		else if(command == "grasp" || command == "place")
		{
			output.addString("Capito");
			yarp::sig::Matrix TL(4,4); TL.eye();
			TL[0][3] = graspPos[0];
			TL[1][3] = graspPos[1];
			TL[2][3] = graspPos[2];
			
			yarp::sig::Matrix TR(4,4); TR.eye();
			TR[0][3] = graspPos[0];
			TR[1][3] =-graspPos[1];
			TR[2][3] = graspPos[2];
			
			controller.move_to_pose(TL*RL, TR*RR);
		}	
		else if(command == "home")					// Return arms to the start position
		{
			output.addString("Casa");
			controller.move_to_position(armHome, armHome, torsoHome);
		}
		else if(command == "in")
		{
			output.addString("Vicino");
			controller.translate(	yarp::sig::Vector({0.0,-0.05, 0.0}),
						yarp::sig::Vector({0.0, 0.05, 0.0}));
		}
		else if(command == "left")					// Move both hands left
		{
			output.addString("Sinistra");
			controller.translate(	yarp::sig::Vector({ 0.0, 0.1, 0.0}),
						yarp::sig::Vector({ 0.0, 0.1, 0.0}));
		}
		else if(command == "left hand pose")				// Print the pose of the left hand
		{
			output.addString("Check the other terminal!");
			controller.print_pose("left");
		}
		else if(command == "manipulability")
		{
			output.addString("Capito");
			controller.set_redundant_task(1, 3.0);
		}
		else if(command == "out")
		{
			output.addString("Lontano");
			controller.translate(	yarp::sig::Vector({ 0.0, 0.05, 0.0}),
						yarp::sig::Vector({ 0.0,-0.05, 0.0}));
		}
		else if(command == "ready")
		{
			output.addString("Pronto");
			controller.move_to_position(shake, shake, torsoHome);
		}
		else if(command == "receive")
		{
			output.addString("Grazie");
			controller.move_to_position(receive, receive, torsoHome);
		}
		else if(command == "right")
		{
			output.addString("Destra");
			controller.translate(	yarp::sig::Vector({ 0.0,-0.1, 0.0}),
						yarp::sig::Vector({ 0.0,-0.1, 0.0}));
		}
		else if(command == "right hand pose")				// Print the pose of the right hand		
		{
			output.addString("Check the other terminal!");
			controller.print_pose("right");
		}
		else if(command == "shake")
		{
			output.addString("Piacere");
			controller.move_to_position(armHome, shake, torsoHome);
		}
		else if(command == "stiffness")
		{
			output.addString("Capito");
			controller.set_redundant_task(2,0.8);
		}
		else if(command == "stop")					// Stop any control threads running
		{
			output.addString("Fermata");
			controller.stop();
		}
		else if(command == "up")
		{
			output.addString("Su");
			controller.translate(	yarp::sig::Vector({ 0.0, 0.0, 0.1}),
						yarp::sig::Vector({ 0.0, 0.0, 0.1}));
		}
		else if(command == "wave")
		{
			output.addString("Ciao");
			controller.move_to_position(armHome, wave, torsoHome);
		}
		else
		{
			output.addString("Cosa");
			controller.stop();					// Stop any movement
		}

		port.reply(output);						// Respond to the user
	}
	
	controller.close();							// Close the device drivers
	
	return 0;								// No problems with main()
}
