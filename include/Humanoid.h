/*
*	Robot control class using iDynTree.
*/

#ifndef HUMANOID_H_
#define HUMANOID_H_

#include <iDynTree/KinDynComputations.h>					// Class that does inverse dynamics calculations
#include <iDynTree/Model/Model.h>						// Class that holds basic kinematic & dynamic info
#include <iDynTree/ModelIO/ModelLoader.h>					// Extracts information from URDF
#include <JointController.h>							// Interfaces with motors on the robot
#include <yarp/os/PeriodicThread.h>						// Keeps timing of the control loop

class Humanoid : public yarp::os::PeriodicThread
{
	public:
		Humanoid(const std::string &fileName);				// Constructor
		
		void close();							// Closes the device drivers for all the limbs
		
		bool update_state();
		
	private:
		bool isValid = true;						// Will not do computations if true
		int dofs;							// Degrees of freedom
		iDynTree::KinDynComputations computer;				// Does all the kinematics & dynamics
		iDynTree::Model model;						// I don't know what this does
		JointController leftArm, rightArm, leftLeg, rightLeg, torso;	// Communicates with all the limbs
		
		// Control loop stuff
		bool threadInit() {return true;}
		void run() {}
		void threadRelease() {}
};										// Semicolon needed after a class declaration

/******************** Constructor *********************/
Humanoid::Humanoid(const std::string &fileName) : yarp::os::PeriodicThread(0.01)
{
	// Load a model
	iDynTree::ModelLoader loader;						// Temporary
	if(!loader.loadModelFromFile(fileName))					// Try to load the model
	{
		std::cerr << "[ERROR] [HUMANOID] Constructor : Could not load model from the path: " << fileName << std::endl;
		this->isValid &= false;
	}
	else									// Successful, now move on to creating the model
	{
		if(!this->computer.loadRobotModel(loader.model()))
		{
			std::cerr << "[ERROR] [HUMANOID] Constructor : Could not generate KinDynComputations class from given model. "
				<< loader.model().toString() << std::endl;
			this->isValid &= false;
		}
		else
		{
			this->model = computer.model();				// This stores basic kinematic & dynamic properties
			this->dofs = model.getNrOfDOFs();			// Get the number of joints
			std::cout << "[INFO] [HUMANOID] Successfully created iDynTree model from " << fileName << "." << std::endl;
			
			// Configure the device drivers for the different body parts
			this->isValid &= this->leftArm.configure_drivers("/local/left_arm", "/icubSim/left_arm", "left arm", 7);
			this->isValid &= this->leftLeg.configure_drivers("/local/left_leg", "/icubSim/left_leg", "left leg", 7);
			this->isValid &= this->rightArm.configure_drivers("/local/right_arm", "/icubSim/right_arm", "right arm", 6);
			this->isValid &= this->rightLeg.configure_drivers("/local/right_leg", "/icubSim/right_leg", "right leg", 6);
			this->isValid &= this->torso.configure_drivers("/local/torso", "/icubSim/torso", "torso", 3);
		}
	}
}

/******************** Close the device drivers on the robot ********************/
void Humanoid::close()
{
	this->leftArm.close();
	this->leftLeg.close();
	this->rightArm.close();
	this->rightLeg.close();
	this->torso.close();
}

/******************** Update the joint state for all the limbs ********************/
bool Humanoid::update_state()
{
	return true;
}

/*
	Eigen::MatrixXd J(6,this->n+6);
	bool OK = computer.getFrameFreeFloatingJacobian(frameIndex, iDynTree::make_matrix:view(J));

	Eigen::MatrixXd M(6+this->n, 6+this->n);
	bool OK = computer.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(M);
*/

#endif
