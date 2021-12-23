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
		
		if(command == "close") 						// Shut down the robot
		{
			output.addString("Arrivederci");
			controller.stop();					// Stop any control threads
			controlActive = false;					// Stop the while loop
		}
		else if(command == "stop")					// Stop any control threads running
		{
			output.addString("Fermata");
			controller.stop();
		}
		else if(command == "left hand")
		{
			output.addString("Check the other terminal!");
			controller.print_pose("left");
		}
		else if(command == "right hand")
		{
			output.addString("Check the other terminal!");
			controller.print_pose("right");
		}
		else if(command == "base")
		{
			output.addString("Check the other terminal!");
			controller.print_pose("base");
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
	yarp::sig::Matrix DL = yarp::math::rpy2dcm(yarp::sig::Vector({-1.15*M_PI/2,-0.0,0})); // Keeps the palms vertical
	yarp::sig::Matrix DR = yarp::math::rpy2dcm(yarp::sig::Vector({-0.85*M_PI/2, 0.0,0}));

	// Joint Control Options
	if(input == "home")
	{
		space = 1;
		response = "Casa";
		torso    = yarp::sig::Vector({0,0,0});
		leftArm  = yarp::sig::Vector({-30, 30,  00,  45,  00,  00,  00});
		rightArm = yarp::sig::Vector({-30, 30,  00,  45,  00,  00,  00});
	}
	
	// Cartesian Control Options
	else if(input == "up")
	{
		space = 2;
		response = "Su";

		yarp::sig::Matrix temp = robot.get_hand_pose("left");
		for(int i = 0; i < 3; i++) DL[i][3] = temp[i][3];		// Assign the translation
		DL[2][3] += 0.1;
		leftHand = DL;
		
		temp = robot.get_hand_pose("right");
		for(int i = 0; i < 3; i++) DR[i][3] = temp[i][3];		// Assign the translation
		DR[2][3] += 0.1;
		rightHand = DR;
		
	}
	else if(input == "down")
	{
		space = 2;
		response = "Giu`";
		
		yarp::sig::Matrix temp = robot.get_hand_pose("left");
		for(int i = 0; i < 3; i++) DL[i][3] = temp[i][3];		// Assign the translation
		DL[2][3] -= 0.1;
		leftHand = DL;
		
		temp = robot.get_hand_pose("right");
		for(int i = 0; i < 3; i++) DR[i][3] = temp[i][3];		// Assign the translation
		DR[2][3] -= 0.1;
		rightHand = DR;
	}
	else if(input == "left")
	{
		space = 2;
		response = "Sinistra";

		yarp::sig::Matrix temp = robot.get_hand_pose("left");
		for(int i = 0; i < 3; i++) DL[i][3] = temp[i][3];		// Assign the translation
		DL[1][3] += 0.1;
		leftHand = DL;
		
		temp = robot.get_hand_pose("right");
		for(int i = 0; i < 3; i++) DR[i][3] = temp[i][3];		// Assign the translation
		DR[1][3] += 0.1;
		rightHand = DR;
	}
	else if(input == "right")
	{
		space = 2;
		response = "Destra";
		
		yarp::sig::Matrix temp = robot.get_hand_pose("left");
		for(int i = 0; i < 3; i++) DL[i][3] = temp[i][3];		// Assign the translation
		DL[1][3] -= 0.1;
		leftHand = DL;
		
		temp = robot.get_hand_pose("right");
		for(int i = 0; i < 3; i++) DR[i][3] = temp[i][3];		// Assign the translation
		DR[1][3] -= 0.1;
		rightHand = DR;
	}
	else if(input == "in")
	{
		space = 2;
		response = "Piu` vicino";
		
		yarp::sig::Matrix temp = robot.get_hand_pose("left");
		for(int i = 0; i < 3; i++) DL[i][3] = temp[i][3];		// Assign the translation
		DL[1][3] -= 0.05;
		leftHand = DL;
		
		temp = robot.get_hand_pose("right");
		for(int i = 0; i < 3; i++) DR[i][3] = temp[i][3];		// Assign the translation
		DR[1][3] += 0.05;
		rightHand = DR;	
	}
	else if(input == "out")
	{
		space = 2;
		response = "Piu` lontano";
		
		yarp::sig::Matrix temp = robot.get_hand_pose("left");
		for(int i = 0; i < 3; i++) DL[i][3] = temp[i][3];		// Assign the translation
		DL[1][3] += 0.05;
		leftHand = DL;
		
		temp = robot.get_hand_pose("right");
		for(int i = 0; i < 3; i++) DR[i][3] = temp[i][3];		// Assign the translation
		DR[1][3] -= 0.05;
		rightHand = DR;
	}	
	else if(input == "forward")
	{
		space = 2;
		response = "Avanti";
		
		yarp::sig::Matrix temp = robot.get_hand_pose("left");
		for(int i = 0; i < 3; i++) DL[i][3] = temp[i][3];		// Assign the translation
		DL[0][3] += 0.1;
		leftHand = DL;
		
		temp = robot.get_hand_pose("right");
		for(int i = 0; i < 3; i++) DR[i][3] = temp[i][3];		// Assign the translation
		DR[0][3] += 0.1;
		rightHand = DR;
	}
	else if(input == "back")
	{
		space = 2;
		response = "Arretrato";
		
		yarp::sig::Matrix temp = robot.get_hand_pose("left");
		for(int i = 0; i < 3; i++) DL[i][3] = temp[i][3];		// Assign the translation
		DL[0][3] -= 0.1;
		leftHand = DL;
		
		temp = robot.get_hand_pose("right");
		for(int i = 0; i < 3; i++) DR[i][3] = temp[i][3];		// Assign the translation
		DR[0][3] -= 0.1;
		rightHand = DR;
	}
	else if(input == "straighten")
	{
		space = 2;
		response = "Ricevuto";
		
		leftHand = robot.get_hand_pose("left");
		leftHand.setSubmatrix(DL.submatrix(0,2,0,2), 0,0);
		
		rightHand = robot.get_hand_pose("right");
		rightHand.setSubmatrix(DR.submatrix(0,2,0,2),0,0);
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
	else if(input == "nonullspace")
	{
		space = 2;
		response = "Ricevuto";
		robot.set_redundant_task(0,0);
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
			// Convert from deg -> rad before sending through
			torso *= M_PI/180;
			leftArm *= M_PI/180;
			rightArm *= M_PI/180;
			robot.move_to_position(leftArm, rightArm, torso);
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
