/*
*	Robot control class using iDynTree.
*/

#ifndef HUMANOID_H_
#define HUMANOID_H_

#include <Eigen/Core>								// Eigen::MatrixXd
#include <iDynTree/KinDynComputations.h>					// Class that does inverse dynamics calculations
#include <iDynTree/Model/FreeFloatingState.h>					// iDynTree::FreeFloatingGeneralizedTorques
#include <iDynTree/Model/Model.h>						// Class that holds basic kinematic & dynamic info
#include <iDynTree/ModelIO/ModelLoader.h>					// Extracts information from URDF
#include <JointInterface.h>							// Interfaces with motors on the robot
#include <yarp/os/PeriodicThread.h>						// Keeps timing of the control loop

std::vector<std::string> jointList = {	// Torso
					  "torso_pitch"
					, "torso_roll"
					, "torso_yaw"
					// Left Arm
					, "l_shoulder_pitch"
					, "l_shoulder_roll"
					, "l_shoulder_yaw"
					, "l_elbow"
					// Right Arm
					, "r_shoulder_pitch"
					, "r_shoulder_roll"
					, "r_shoulder_yaw"
					, "r_elbow"
					// Left Leg
					, "l_hip_pitch"
					, "l_hip_roll"
					, "l_hip_yaw"
					, "l_knee"
					, "l_ankle_pitch"
					, "l_ankle_roll"
					// Right Leg
					, "r_hip_pitch"
					, "r_hip_roll"
					, "r_hip_yaw"
					, "r_knee"
					, "r_ankle_pitch"
					, "r_ankle_roll"};
					// Neck
//					, "neck_pitch"
//					, "neck_roll"
//					, "neck_yaw"};
					//, "neck_fixed_joint"};

class Humanoid	: public yarp::os::PeriodicThread
		, public JointInterface
{
	public:
		Humanoid(const std::string &fileName);				// Constructor
		
		bool update_state();	
		bool move_to_position(const iDynTree::VectorDynSize &position);
		bool move_to_pose(const iDynTree::Transform &pose, const std::string &whichHand);
		bool move_to_pose(const iDynTree::Transform &leftHand, const iDynTree::Transform &rightHand);
		
	private:
		bool isValid = true;						// Will not do computations if true
		double Kq = 50; 
		double Kd = 2*sqrt(Kq);
		
		double startTime;
		enum ControlSpace {joint, cartesian, dual} controlSpace;
		int n;								// Degrees of freedom
		iDynTree::KinDynComputations computer;				// Does all the kinematics & dynamics
		iDynTree::Model model;						// I don't know what this does
		iDynTree::Transform torsoPose;
		iDynTree::Twist torsoTwist;
		iDynTree::Vector3 gravity;
		Quintic jointTrajectory;					// Joint level control
		
		// Control loop stuff
		bool threadInit();
		void run();
		void threadRelease();
		
		void print_kinematics();
};										// Semicolon needed after a class declaration

/******************** Constructor *********************/
Humanoid::Humanoid(const std::string &fileName)
	: yarp::os::PeriodicThread(0.01)
	, JointInterface(jointList)
	, torsoPose(iDynTree::Transform::Identity())
	, torsoTwist(iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)))
{
	// Set the gravity vector
	this->gravity(0) = 0.0;
	this->gravity(1) = 0.0;
	this->gravity(2) =-9.81;
	
	// Load a model
	iDynTree::ModelLoader loader;						// Temporary

//	if(!loader.loadModelFromFile(fileName))
	if(!loader.loadReducedModelFromFile(fileName, jointList, "urdf"))
	{
		std::cerr << "[ERROR] [HUMANOID] Constructor: Could not load model from path " << fileName << std::endl;
		this->isValid = false;
	}
	else
	{
		if(!this->computer.loadRobotModel(loader.model()))
		{
			std::cerr << "[ERROR] [HUMANOID] Constructor : Could not generate iDynTree::KinDynComputations class from given model: "
				<< loader.model().toString() << std::endl;
		}
		else
		{
			this->model = computer.model();
			this->n = model.getNrOfDOFs();
			
			std::cout << "[INFO] [HUMANOID] Successfully created iDynTree model from " << fileName << "." << std::endl;
	
			// print_kinematics()
		}
	}
}

/******************** Update the joint state for all the limbs ********************/
bool Humanoid::update_state()
{
	if(JointInterface::read_encoders())
	{
		if(this->computer.setRobotState(this->torsoPose, this->q, this->torsoTwist, this->qdot, this->gravity))
		{
			return true;
		}
		else
		{
			std::cerr << "[ERROR] [HUMANOID] update_state() : Could not set state for the iDynTree::iKinDynComputations object." << std::endl;
			return false;
		}
	}
	else
	{
		std::cerr << "[ERROR] [HUMANOID] update_state() : Could not update state from the JointInterface class." << std::endl;
		return false;
	}
}

/******************** Move the joints to a desired position ********************/
bool Humanoid::move_to_position(const iDynTree::VectorDynSize &position)
{
	if(position.size() > this->n)
	{
		std::cerr << "[ERROR] [HUMANOID] move_to_position() : Input vector has " << position.size() 
			<< " elements, but there are only " << this->n << " joints in this model." << std::endl;
		return false;
	}
	else
	{
		stop();									// Stop any control threads that are running
		
		this->controlSpace = joint;						// Set the control space	
			
		iDynTree::VectorDynSize desired = this->q;				
		
		// Ensure that the target is within joint limits
		for(int i = 0; i < position.size(); i++)
		{
			if(position[i] >= this->qMax[i]) 	desired[i] = this->qMax[i] - 0.001; // Just below the limit
			else if(position[i] <= this->qMin[i])	desired[i] = this->qMin[i] + 0.001; // Just above the limit
			else					desired[i] = position[i];
		}
		
		// Compute optimal time scaling
		double dt, dq;
		double endTime = 2.0;
		for(int i = 0; i < this->n; i++)
		{
			dq = abs(desired[i] - this->q[i]);				// Distance to target
			if(dq > 0) dt = (15*dq)/(8*this->vLim[i]);			// Time to reach target at peak velocity
			if(dt > endTime) endTime = dt;					// If slowes time so far, override
		}
		
		this->jointTrajectory = Quintic(desired, this->q, 0, endTime);		// Create new joint trajectory
		
		start(); 								// Go immediately to threadInit()
		
		return true;
	}
}

/******************** This is executed just after start() is called ********************/
bool Humanoid::threadInit()
{
	this->startTime = yarp::os::Time::now();
	return true;
	
	// Go immediately to run()
}

/******************** MAIN CONTROL LOOP ********************/
void Humanoid::run()
{
	update_state();									// Update the joint state information
	double elapsedTime = yarp::os::Time::now() - this->startTime;			// Get elapsed time since start
	
	switch(this->controlSpace)
	{
		case joint:
		{
			std::cout << "\nWorker bees can leave." << std::endl;
			std::cout << "Even drones can fly away." << std::endl;
			std::cout << "The Queen is their slave.\n" << std::endl;
						
			// Get the desired joint state
			iDynTree::VectorDynSize q_d(this->n), qdot_d(this->n), qddot_d(this->n);
			this->jointTrajectory.get_state(q_d, qdot_d, qddot_d, elapsedTime);
					
			iDynTree::FreeFloatingGeneralizedTorques grav(this->model);	// Gravitational force object for given model structure
			this->computer.generalizedGravityForces(grav);			// Compute the gravitational forces and torques
			iDynTree::VectorDynSize g = grav.jointTorques();		// Extract only the joint torques
			
			iDynTree::VectorDynSize tau(this->n);
			
			for(int i = 0; i < this->n; i++)
			{
				tau(i) = g(i) + this->Kd*(qdot_d(i) - this->qdot(i)) + this->Kq*(q_d(i) - this->q(i));
			}
			
			break;
		}
		case cartesian:
		{
			
			break;
		}
		case dual:
		{
			
			break;
		}
		default:
		{
		
		}
	}
	
	// Run this function repeatedly until stop() is called
}

/******************** This is executed just after stop() is called ********************/
void Humanoid::threadRelease()
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

/******************** Prints out stuff for debugging purposes ********************/
void Humanoid::print_kinematics()
{		
	std::cout << "\nHere is the given joint list:" << std::endl;
	for(int i = 0; i < jointList.size(); i++) std::cout << jointList[i] << std::endl;

	std::cout << "\nHere is the joint list in the model:" << std::endl;
	for(int i = 0; i < this->n; i++) std::cout << this->model.getJointName(i) << std::endl;

	if(!update_state()) std::cerr << "[ERROR] [HUMANOID] Unable to update state." << std::endl;
	else
	{
		// Print out the pose of the left hand
		iDynTree::Transform T = this->computer.getWorldTransform("l_hand");
		std::cout << "\nHere is the pose of the left hand:" << std::endl;
		std::cout << T.asHomogeneousTransform().toString() << std::endl;
		
		// Print out the Jacobian for the left hand
		Eigen::MatrixXd J(6, this->n + 6);
		if(this->computer.getFrameFreeFloatingJacobian("l_hand", J))
		{
			std::cout << "\nHere is the Jacobian to the left hand:" << std::endl;
			std::cout << J << std::endl;
		}
		else std::cerr << "[ERROR] [HUMANOID] Something went wrong. Cannot compute kinematics & dynamics." << std::endl;
		
		T = this->computer.getWorldTransform("r_hand");
		std::cout << "\nHere is the pose of the right hand:" << std::endl;
		std::cout << T.asHomogeneousTransform().toString() << std::endl;
		
		// Print out the Jacobian for the right hand:
		if(this->computer.getFrameFreeFloatingJacobian("r_hand", J))
		{
			std::cout << "\nHere is the Jacobian to the right hand:" << std::endl;
			std::cout << J << std::endl;
		}
		else std::cerr << "[ERROR] [HUMANOID] Something went wrong. Cannot compute the kinematics & dynamics." << std::endl;
	}
}

#endif
