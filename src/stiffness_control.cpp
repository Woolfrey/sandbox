#include <StiffnessCtrl.h>
#include <yarp/os/RpcServer.h>

// Pre-defined joint positions
yarp::sig::Vector armHome( {-00, 40, -20,  135,  00,  00,  0});
yarp::sig::Vector receive( {-50, 10,  00,  40, -60,  20, 10});
yarp::sig::Vector shake(   {-50, 40,  65,  45, -70, -20,  0});
yarp::sig::Vector torsoHome({0.0, 0.0, 0.0});
yarp::sig::Vector wave(    {-30, 50, -30, 105,  30,  00, 00});
yarp::sig::Vector straight({ 00, 20,  00,  20,  00,  00, 00});

/********************* This is where the action happens *********************/
int main(int argc, char *argv[])
{
	// Convert vectors from deg to rad because YARP is annoying
	armHome *= M_PI/180;
	receive *= M_PI/180;
	shake *= M_PI/180;
	torsoHome *= M_PI/180;
	wave *= M_PI/180;
	straight *= M_PI/180;
	
	yarp::os::Network yarp;							// Connect to the yarp connectwork
	
	StiffnessCtrl controller(100);						// Create dual arm controller object
	
//	controller.set_joint_limits(3, -5, 135, "right");			// New joint limits for elbow joint on the right arm

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
		else if(command == "home")					// Return arms to the start position
		{
			output.addString("Casa");
			controller.move_to_position(straight, armHome, torsoHome);
		}
		else if(command == "shake")
		{
			output.addString("Piacere");
			controller.move_to_position(armHome, shake, torsoHome);
		}
		else if(command == "start")
		{
			output.addString("Pronto");
			yarp::sig::Matrix T(4,4); T.eye();
			T[0][3] = 0.2;
			T[1][3] =-0.05;
			T[2][3] = 0.85;
			yarp::sig::Matrix rot = yarp::math::rpy2dcm({-M_PI,0.0,-M_PI/2});
			yarp::sig::Matrix blah;	// Empty matrix
			controller.move_to_pose(T*rot);
		}
		else if(command == "stop")					// Stop any control threads running
		{
			output.addString("Fermata");
			controller.stop();
		}
		else if(command == "wave")
		{
			output.addString("Ciao");
			controller.move_to_position(armHome, wave, torsoHome);
		}
		else if(command == "x" || command == "y" || command == "z")
		{
			output.addString("Capito");
			controller.set_direction(command);
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
