#include <ArmController.h>									// This handles the mid-level kinematics
class DualArmController
{
	public:
		DualArmController();								// Constructor
		
		void close();									// Close all device drivers
		
		void move_to_position(	yarp::sig::Vector &left,
					yarp::sig::Vector &right);
		
		yarp::sig::Vector get_positions(const std::string &which);
		
	private:
		ArmController leftArm, rightArm;
		MultiJointController torso;

};												// Semicolon needed after class declaration


/******************** Move the joints to a given position ********************/
void DualArmController::move_to_position(yarp::sig::Vector &left, yarp::sig::Vector &right)
{
	this->leftArm.move_to_position(left);
	this->rightArm.move_to_position(right);
}

/******************** Get the joint positions ********************/
yarp::sig::Vector DualArmController::get_positions(const std::string &which)
{
	if (which.compare("left")) 	return this->leftArm.get_positions();
	else				return this->rightArm.get_positions();
}

/******************** Close the device drivers ********************/
void DualArmController::close()
{
	this->leftArm.close();
	this->rightArm.close();
	this->torso.close();
}

/******************** Empty constructor ********************/
DualArmController::DualArmController()
{
	// Configure the left arm
	std::string local = "/local/left";
	std::string remote = "/icubSim/left_arm";
	std::string name = "left";
	this->leftArm.configure(local, remote, name, 16);					// Arm: 1 - 7, Hand: 8 - 16
	
	// Configure the right arm
	local = "/local/right";
	remote = "/icubSim/right_arm";
	name = "right";
	this->rightArm.configure(local, remote, name, 16);					// Arm: 1 - 7, Hand: 8 - 16
	
	// Configure the torso
	local = "/local/torso";
	remote = "/icubSim/torso";
	name = "torso";
	this->torso.configure_drivers(local, remote, name, 3);
}

/******************** Move the joints back to their initial configuration ********************
void DualArmController::move_to_start_position()
{
	yarp::sig::Vector left_target({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	yarp::sig::Vector right_target({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	yarp::sig::Vector torso_target({0.0, 0.0, 0.0});
	
	this->leftArm->move_to_position(left_target);
	this->rightArm->move_to_position(right_target);
	this->torso->move_to_position(torso_target);
}

/******************** Move one of the hands in different directions ********************
void DualArmController::move_left()
{

}
void DualArmController::move_right()
{

}

*/
