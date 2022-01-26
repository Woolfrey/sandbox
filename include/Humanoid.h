/*
*	Robot control class using iDynTree.
*/

/*
	Eigen::MatrixXd J(6,this->n+6);
	bool OK = computer.getFrameFreeFloatingJacobian(frameIndex, iDynTree::make_matrix:view(J));

	Eigen::MatrixXd M(6+this->n, 6+this->n);
	bool OK = computer.getFreeFloatingMassMatrix(iDynTree::make_matrix_view(M);
*/

/* Joint List:

// Left arm
"l_shoulder_pitch";
"l_shoulder_roll";
"l_shoulder_yaw";
"l_elbow";

// Right Arm
"r_shoulder_pitch";
"r_shoulder_roll";
"r_shoulder_yaw";
"r_elbow";

// Left Leg
"l_hip_pitch";
"l_hip_roll";
"l_hip_yaw";
"l_knee";
"l_ankle_pitch";
"l_ankle_roll";

// Right Leg
"r_hip_pitch";
"r_hip_roll";
"r_hip_yaw";
"r_knee";
"r_ankle_pitch";
"r_ankle_roll";

// Torso
"torso_pitch";
"torso_roll";
"torso_yaw";

// Head
"neck_pitch";
"neck_roll";
"neck_yaw";
"neck_fixed_joint";

// Sensors
r_leg_ft_sensor
r_foot_rear_ft_sensor
r_foot_front_ft_sensor
l_arm_ft_sensor
r_arm_ft_sensor
l_leg_ft_sensor
l_foot_front_ft_sensor
l_foot_rear_ft_sensor




*/

#ifndef HUMANOID_H_
#define HUMANOID_H_

#include <Eigen/Core>								// Eigen::MatrixXd
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
		JointController leftArm, rightArm, leftLeg, rightLeg, torso;	// Communicates with the joints on all the limbs
		
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
	
	//loader.loadReducedFromFile(fileName, std::vector<std::string> jointList, const std::string filtype = "");
	
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
			
			// Test the Kinematics
			if(this->isValid && update_state())
			{
				// Print out all the link and joint names
				std::cout << "\nHere are the links:" << std::endl;
				for(int i = 0; i < model.getNrOfLinks(); i++)
				{
					std::cout << this->model.getLinkName(i) << std::endl;
				}
				
				std::cout << "\nHere are the joints:" << std::endl;
				for(int i = 0; i < model.getNrOfJoints(); i++)
				{
					std::cout << this->model.getJointName(i) << std::endl;
				}
				
				// Print out the pose of the left hand
				iDynTree::Transform T = this->computer.getWorldTransform("l_hand");
				std::cout << "\n[INFO] [HUMANOID] Here is the pose of the left hand:" << std::endl;
				std::cout << T.toString() << std::endl;
			
				// Print out the Jacobian for the left hand
				Eigen::MatrixXd J(6, this->dofs + 6);
				if(this->computer.getFrameFreeFloatingJacobian("l_hand", J))
				{
					std::cout << "\n[INFO] [HUMANOID] Here is the Jacobian to the left hand:" << std::endl;
					std::cout << J << std::endl;
				}
				else	std::cerr << "[ERROR] [HUMANOID] Could not obtain the Jacobian for the left hand." << std::endl;
			}
			else
			{
				std::cout << "[INFO] [HUMANOID] Something went wrong. Cannot compute kinematics & dynamics." << std::endl;
			}	
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
	bool success = true;
	
	// Read the encoders on all the limbs
	success &= this->leftArm.read_encoders();
	success &= this->rightArm.read_encoders();
	success &= this->leftLeg.read_encoders();
	success &= this->rightLeg.read_encoders();
	success &= this->torso.read_encoders();
	
	// Compile the joint state vectors
	
	// iDynTree::Transform P;						// Base pose
	// iDynTree::VectorDynSize q;						// Joint positions
	// iDynTree::Twist v;							// Base velocity
	// iDynTree::VectorDynSize qdot;					// Joint velocities
	// iDynTree::Vector3 g;							// Gravitational acceleration
	
	// computer.setRobotState(P, q, v, qdot, g);
	
	return success;
}

#endif
