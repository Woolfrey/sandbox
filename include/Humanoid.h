/*
*	Robot control class using iDynTree.
*/

#ifndef HUMANOID_H_
#define HUMANOID_H_

#include <CartesianTrajectory.h>						// Custom trajectory class
#include <Eigen/Core>								// Eigen::MatrixXd
#include <Haiku.h>
#include <iDynTree/KinDynComputations.h>					// Class that does inverse dynamics calculations
#include <iDynTree/Model/FreeFloatingState.h>					// iDynTree::FreeFloatingGeneralizedTorques
#include <iDynTree/Model/Model.h>						// Class that holds basic kinematic & dynamic info
#include <iDynTree/ModelIO/ModelLoader.h>					// Extracts information from URDF
#include <JointInterface.h>							// Interfaces with motors on the robot
#include <Quintic.h>								// Custom trajectory class
#include <yarp/os/PeriodicThread.h>						// Keeps timing of the control loop

std::vector<double> startConfiguration({ 00.00,  00.00,  00.00,									// Torso
					-30.00*M_PI/180,  30.00*M_PI/180,  00.00,  45.00*M_PI/180,  00.00,  00.00,  00.00,	// Left arm
					-30.00*M_PI/180,  30.00*M_PI/180,  00.00,  45.00*M_PI/180,  00.00,  00.00,  00.00});	// Right arm

std::vector<std::string> jointList = {// Torso
					  "torso_pitch"
					, "torso_roll"
					, "torso_yaw"
					// Left Arm
					, "l_shoulder_pitch"
					, "l_shoulder_roll"
					, "l_shoulder_yaw"
					, "l_elbow"
					, "l_wrist_prosup"
					, "l_wrist_pitch"
					, "l_wrist_yaw"
					// Right Arm
					, "r_shoulder_pitch"
					, "r_shoulder_roll"
					, "r_shoulder_yaw"
					, "r_elbow"
					, "r_wrist_prosup"
					, "r_wrist_pitch"
					, "r_wrist_yaw"};
					// Left Leg
//					, "l_hip_pitch"
//					, "l_hip_roll"
//					, "l_hip_yaw"
//					, "l_knee"
//					, "l_ankle_pitch"
//					, "l_ankle_roll"
					// Right Leg
//					, "r_hip_pitch"
//					, "r_hip_roll"
//					, "r_hip_yaw"
//					, "r_knee"
//					, "r_ankle_pitch"
//					, "r_ankle_roll"};
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
		void halt();							// Stop any control and maintain current position
		
		void force_test();
		
	private:
		bool isValid = true;						// Will not do computations if true
		enum ControlSpace {joint, cartesian, dual} controlSpace;
		
		iDynTree::FreeFloatingGeneralizedTorques generalizedForces;	// Forces and torques
		
		// Joint Control
		double Kq = 50;							// Proportional gain
		double Kd = 3.0;						// Derivative gain
		Quintic jointTrajectory;					// Joint level control
		
		// Cartesian Control
		CartesianTrajectory cartesianTrajectory;
		iDynTree::MatrixDynSize K;
		iDynTree::MatrixDynSize D;
		
		// Kinematics & dynamics
		int n;								// Degrees of freedom
		iDynTree::KinDynComputations computer;				// Does all the kinematics & dynamics
		iDynTree::Model model;						// I don't know what this does
		iDynTree::Transform torsoPose;
		iDynTree::Twist torsoTwist;
		iDynTree::Vector3 gravity;
		
		// Functions
		void print_kinematics();					// Used for debugging
		
		// Control loop stuff
		double startTime;						// Used to time the control loop
		bool threadInit();						// From yarp::os::PeriodicThread class
		void run();							// From yarp::os::PeriodicThread class
		void threadRelease();						// From yarp::os::PeriodicThread class
		
};										// Semicolon needed after a class declaration

/////////////////////////////////////////////////////////////////////////////////////////////////////
//		   			Constructor 						  //
////////////////////////////////////////////////////////////////////////////////////////////////////
Humanoid::Humanoid(const std::string &fileName)
	: yarp::os::PeriodicThread(0.01)					// Create the threading object for control
	, JointInterface(jointList)						// Open communication with the robot
	, torsoPose(iDynTree::Transform::Identity())				// Set the default pose for the model
	, torsoTwist(iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)))
{
	// Set the gravity vector
	this->gravity(0) = 0.0;
	this->gravity(1) = 0.0;
	this->gravity(2) =-9.81;
	
	// Load a model
	iDynTree::ModelLoader loader;						// Temporary

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
			this->model = computer.model();				// Get the model from the computer
			this->n = model.getNrOfDOFs();				// Degrees of freedom / number of joints
			this->generalizedForces.resize(this->model);		// Restructure the GeneralizedTorque class to match the model
			
			std::cout << "[INFO] [HUMANOID] Successfully created iDynTree model from " << fileName << "." << std::endl;
	
			update_state();						// Get the current joint state
			if(activate_control())
			{
				move_to_position(iDynTree::VectorDynSize(startConfiguration));
			}
			else	std::cerr << "[ERROR] [HUMANOID] Constructor: Could not activate joint control." << std::endl;
		}
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//				Update the joint state for all the limbs			  //
////////////////////////////////////////////////////////////////////////////////////////////////////
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

/////////////////////////////////////////////////////////////////////////////////////////////////////
//				Move the joints to a desired position	 			  //
////////////////////////////////////////////////////////////////////////////////////////////////////
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
		stop();								// Stop any control threads that are running
		this->controlSpace = joint;					// Set the control space		
		iDynTree::VectorDynSize desired = this->q;			// Assign as current joint state, override later...			
		
		// Ensure that the target is within joint limits
		bool warning = false;
		for(int i = 0; i < position.size(); i++)
		{
			if(position[i] >= this->qMax[i])
			{
				desired[i] = this->qMax[i] - 0.001;		// Just below the limit
				warning = true;
			}
			else if(position[i] <= this->qMin[i])
			{
				desired[i] = this->qMin[i] + 0.001;		// Just above the limit
				warning = true;
			}
			else	desired[i] = position[i];			// Override position
		}
		
		if(warning)
		{
			std::cerr << "[WARNING] [HUMANOID] move_to_position() : Target joint configuration outside of "
				<< "one or more joint limits. Target has been automatically overidden." << std::endl;
		}
		
		// Compute optimal time scaling
		double dt, dq;
		double endTime = 2.0;
		for(int i = 0; i < this->n; i++)
		{
			dq = abs(desired[i] - this->q[i]);			// Distance to target
			if(dq > 0) dt = (15*dq)/(8*this->vLim[i]);		// Time to reach target at peak velocity
			if(dt > endTime) endTime = dt;				// If slowes time so far, override
		}
		
		this->jointTrajectory = Quintic(this->q, desired, 0, endTime);	// Create new joint trajectory
		
		start(); 							// Go immediately to threadInit()
		
		return true;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//				Hold the current joint positions				  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::halt()
{
	if(isRunning())	stop();							// Stop any control threads that might be running
	update_state();								// Get the current joint state
	move_to_position(this->q);						// Run the control to stay at current configuration	
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//				Testing Cartesian force control					  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::force_test()
{
	if(isRunning()) stop();
	this->controlSpace = cartesian;
	start();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//			This is executed just after 'start()' is called				  //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::threadInit()
{
	this->startTime = yarp::os::Time::now();
	return true;
// 	run();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//					MAIN CONTROL LOOP					  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::run()
{
	update_state();									// Update the joint state information
	double elapsedTime = yarp::os::Time::now() - this->startTime;			// Get elapsed time since start
	
	iDynTree::VectorDynSize tau(this->n);						// We want to compute this

	switch(this->controlSpace)
	{
		case joint:
		{
			iDynTree::VectorDynSize qddot(this->n); 			// We want to compute this
			
			// Get the desired joint state
			iDynTree::VectorDynSize q_d(this->n), qdot_d(this->n), qddot_d(this->n), e(this->n);
			this->jointTrajectory.get_state(q_d, qdot_d, qddot_d, elapsedTime);
			
			for(int i = 0; i < this->n; i++)
			{
				qddot[i] = qddot_d[i] + this->Kq*(q_d[i] - this->q[i]) + this->Kd*(qdot_d[i] - this->qdot[i]);
			}
			
			// Compute the inverse dynamics from the joint accelerations
			iDynTree::Vector6 baseAcc; baseAcc.zero();						// Don't move the base
			iDynTree::LinkNetExternalWrenches wrench(this->model); wrench.zero();			// No external forces applied
			this->computer.inverseDynamics(baseAcc, qddot, wrench, this->generalizedForces);	// Solve the inverse dynamics
			tau = this->generalizedForces.jointTorques();						// Get the joint torques
			
			break;
		}
		case cartesian:
		{
			// Compute the gravity compensation
			this->computer.generalizedGravityForces(this->generalizedForces);
			tau = this->generalizedForces.jointTorques();
		
			// Compute the torques from forces at the hand
			Eigen::MatrixXd J(6, 6 + this->n);
			this->computer.getFrameFreeFloatingJacobian("l_hand", J);
			J = J.block(0,6,6,10); 									// Just the Jacobian for the hand

			Eigen::VectorXd f(6);
			f << 0, -2, -2, 0, 0, 0;								// Force vector	
			Eigen::VectorXd temp = J.transpose()*f;

			for(int i = 0; i < 10; i++) tau[i] += temp[i];		
/*
			// Compute the inertia matrix
			Eigen::MatrixXd M(6+this->n, 6+this->n);
			this->computer.getFreeFloatingMassMatrix(M);
			std::cout << "\nHere is the inertia matrix:" << std::endl;
			std::cout << M << std::endl;
			
			// Compute Coriolis & gravity
			this->computer.generalizedBiasForces(this->generalizedForces);
			iDynTree::VectorDynSize h = this->generalizedForces.jointTorques();
			std::cout << "\nHere is the feedback linearization term:" << std::endl;
			std::cout << h.toString() << std::endl;
			iDynTree::Vector6 Jdotqdot = this->computer.getFrameBiasAcc("l_hand");
			std::cout<< Jdotqdot.toString() << std::endl;
*/
			break;
		}
		default:
		{
			haiku();
		}
	}

	send_torque_commands(tau);
		
//	if(stop()) threadRelease();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//			This is executed just after 'stop()' is called				  //
////////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::threadRelease()
{
	this->computer.generalizedGravityForces(this->generalizedForces);		// Get the torque needed to withstand gravity
	send_torque_commands(this->generalizedForces.jointTorques());			// Send to the robot
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
//				Prints out stuff for debugging purposes				  //
////////////////////////////////////////////////////////////////////////////////////////////////////
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
