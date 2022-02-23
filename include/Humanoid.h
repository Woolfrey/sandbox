    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                                  A control class for the iCub robot                            //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef HUMANOID_H_
#define HUMANOID_H_

#include <CartesianTrajectory.h>                                                                   // Custom trajectory class
#include <Cubic.h>
#include <Eigen/Dense>                                                                             // Eigen::MatrixXd
#include <Haiku.h>
#include <iDynTree/KinDynComputations.h>                                                           // Class that does inverse dynamics calculations
#include <iDynTree/Model/FreeFloatingState.h>                                                      // iDynTree::FreeFloatingGeneralizedTorques
#include <iDynTree/Model/Model.h>                                                                  // Class that holds basic kinematic & dynamic info
#include <iDynTree/ModelIO/ModelLoader.h>                                                          // Extracts information from URDF
#include <JointInterface.h>                                                                        // Interfaces with motors on the robot
#include <Quintic.h>                                                                               // Custom trajectory class
#include <yarp/os/PeriodicThread.h>                                                                // Keeps timing of the control loop

std::vector<double> startConfiguration({ 00.00,  00.00,  00.00,                                                              // Torso
					-30.00*M_PI/180,  30.00*M_PI/180,  00.00,  45.00*M_PI/180,  00.00,  00.00,  00.00,   // Left arm
					-30.00*M_PI/180,  30.00*M_PI/180,  00.00,  45.00*M_PI/180,  00.00,  00.00,  00.00}); // Right arm

std::vector<std::string> jointList = {  // Torso
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

class Humanoid : public yarp::os::PeriodicThread,
	         public JointInterface
{
	public:
		Humanoid(const std::string &fileName);                                             // Constructor
		
		// Get Functions
		iDynTree::Transform get_hand_pose(const std::string &whichHand);
		iDynTree::Vector6 get_pose_error(const iDynTree::Transform &desired, const iDynTree::Transform &actual);
		
		// Control Functions
		bool update_state();	
		bool move_to_position(const iDynTree::VectorDynSize &position);
		bool move_to_pose(const iDynTree::Transform &pose, const std::string &whichHand);
		bool move_to_pose(const iDynTree::Transform &leftHand, const iDynTree::Transform &rightHand);
		void halt();                                                                       // Stop any control and maintain current position
		void force_test();							          
		
	private:
		bool isValid = true;                                                               // Will not do computations if true
		enum ControlSpace {joint, cartesian, dual} controlSpace;
		
		iDynTree::FreeFloatingGeneralizedTorques generalizedForces;                        // Forces and torques
		
		// Joint Control
		double Kq = 50;                                                                    // Proportional gain
		double Kd = 3.0;                                                                   // Derivative gain
		Cubic jointTrajectory;                                                             // Joint level control
		
		// Cartesian Control
		CartesianTrajectory leftHandTrajectory, rightHandTrajectory;                       // Internal Cartesian trajectory generatory
		iDynTree::MatrixDynSize K;                                                         // Cartesian stiffness
		iDynTree::MatrixDynSize D;                                                         // Cartesian damping
		
		// Kinematics & dynamics
		int n;                                                                             // Degrees of freedom
		iDynTree::KinDynComputations computer;                                             // Does all the kinematics & dynamics
		iDynTree::Model model;                                                             // I don't know what this does
		iDynTree::Transform torsoPose;
		iDynTree::Twist torsoTwist;
		iDynTree::Vector3 gravity;
		
		// Functions
		Eigen::MatrixXd inverse(const Eigen::MatrixXd &A);                                 // Get the inverse of the given matrix
		void print_kinematics();                                                           // Used for debugging
		void print_dynamics();                                                             // Used for debugging
		
		// Control loop stuff
		double startTime;                                                                  // Used to time the control loop
		bool threadInit();                                                                 // From yarp::os::PeriodicThread class
		void run();                                                                        // From yarp::os::PeriodicThread class
		void threadRelease();                                                              // From yarp::os::PeriodicThread class
		
};                                                                                                 // Semicolon needed after a class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
Humanoid::Humanoid(const std::string &fileName) :
	           yarp::os::PeriodicThread(0.01),                                                 // Create the threading object for control
                   JointInterface(jointList),                                                      // Open communication with the robot
                   torsoPose(iDynTree::Transform::Identity()),                                     // Set the default pose for the model
                   torsoTwist(iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)))
{
	// Set the gravity vector
	this->gravity(0) = 0.0;
	this->gravity(1) = 0.0;
	this->gravity(2) =-9.81;
	
	// Load a model
	iDynTree::ModelLoader loader;                                                              // Temporary

	if(!loader.loadReducedModelFromFile(fileName, jointList, "urdf"))
	{
		std::cerr << "[ERROR] [HUMANOID] Constructor: Could not load model from path " << fileName << std::endl;
		this->isValid = false;
	}
	else
	{
		if(!this->computer.loadRobotModel(loader.model()))
		{
			std::cerr << "[ERROR] [HUMANOID] Constructor : Could not generate iDynTree::KinDynComputations "
			          << "class from given model: " << loader.model().toString() << std::endl;
		}
		else
		{
			this->model = computer.model();                                            // Get the model from the computer
			this->n = model.getNrOfDOFs();                                             // Degrees of freedom / number of joints
			this->generalizedForces.resize(this->model);                               // Restructure force class to match model
			
			std::cout << "[INFO] [HUMANOID] Successfully created iDynTree model from " << fileName << "." << std::endl;
	
			update_state();                                                            // Get the current joint state
			if(activate_control())
			{
				move_to_position(iDynTree::VectorDynSize(startConfiguration));
			}
			else	std::cerr << "[ERROR] [HUMANOID] Constructor: Could not activate joint control." << std::endl;
		}
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Get the pose of one of the hands                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
iDynTree::Transform Humanoid::get_hand_pose(const std::string &whichHand)
{
	if(whichHand == "left")       return this->computer.getWorldTransform("l_hand");
	else if(whichHand == "right") return this->computer.getWorldTransform("r_hand");
	else
	{
		std::cerr << "[ERROR] [HUMANOID] get_hand_pose() : String input was " << whichHand
			  << " but expected 'left' or 'right' as an argument. Returning left hand pose by default." << std::endl;
		return this->computer.getWorldTransform("l_hand");
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Compute the error between 2 poses for feedback purposes                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
iDynTree::Vector6 Humanoid::get_pose_error(const iDynTree::Transform &desired,
                                           const iDynTree::Transform &actual)
{
	iDynTree::Vector6 error;                                                                   // Value to be returned
	
	return error;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Update the joint state for all the limbs                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
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
			std::cerr << "[ERROR] [HUMANOID] update_state() : Could not set state for the "
			          << " iDynTree::iKinDynComputations object." << std::endl;
			return false;
		}
	}
	else
	{
		std::cerr << "[ERROR] [HUMANOID] update_state() : Could not update state from the JointInterface class." << std::endl;
		return false;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Move the joints to a desired configuration                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
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

		if(isRunning()) stop();                                                            // Stop any control threads that are running
		this->controlSpace = joint;                                                        // Set the control space		
		iDynTree::VectorDynSize desired = this->q;                                         // Assign as current joint state, override later...
				
		// Ensure that the target is within joint limits
		bool warning = false;
		std::vector<int> jointNum;                                                         // Store which joints violate limits
		
		for(int i = 0; i < position.size(); i++)
		{
			if(position[i] >= this->qMax[i])
			{
				desired[i] = this->qMax[i] - 0.001;                                // Just below the limit
				jointNum.push_back(i);                                             // Add to list
				warning = true;
			}
			else if(position[i] <= this->qMin[i])
			{
				desired[i] = this->qMin[i] + 0.001;                                // Just above the limit
				jointNum.push_back(i);                                             // Add to list
				warning = true;
			}
			else	desired[i] = position[i];                                          // Override position
		}
		
		if(warning)
		{
			std::cerr << "[WARNING] [HUMANOID] move_to_position() : The target positions for the following "
				  << "joints were outside limits: ";
			for(int i = 0; i < jointNum.size(); i++) std::cout << " Joint No.: " << jointNum[i] << " Target: " << position[i] << ". ";
			std::cout << "Values were automatically overridden." << std::endl;
		}
		
		// Compute "optimal" time scaling (really a heuristic method; true optimal method requires too much math)
		double dt = 2.0;
		double dq;
		for(int i = 0; i < this->n; i++)
		{
			dq = abs(desired[i] - this->q[i]);                                         // Distance to target
			if(dt < 1.5*dq/this->vLim[i]) dt = 2*dq/this->vLim[i];                     // Average velocity >= speed limit
		}

		// Put data in to vectors and pass to Cubic trajectory object
		std::vector<iDynTree::VectorDynSize> points;
		points.push_back(this->q);
		points.push_back(position);
		std::vector<double> times;
		times.push_back(0.0);
		times.push_back(dt);
		this->jointTrajectory = Cubic(points,times);
		
		start();                                                                           // Go immediately to threadInit()
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Move one hand to a desired pose                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_pose(const iDynTree::Transform &pose, const std::string &whichHand)
{
	if(whichHand == "left")
	{
		if(isRunning()) stop();                                                            // Stop any control threads that are running
		this->controlSpace = cartesian;                                                    // Switch to Cartesian control
		
		// TO DO: CHECK THE POSE IS WITHIN THE WORKSPACE
		// TO DO: COMPUTE AN 'OPTIMAL' TRAJECTORY TIME
		double endTime = 5.0;

		iDynTree::Transform T = this->computer.getWorldTransform("l_hand");                // Get the current pose of the left hand
		this->leftHandTrajectory = CartesianTrajectory(T, pose, 0.0, endTime);             // Move left hand from current pose to given pose
		
		T = this->computer.getWorldTransform("r_hand");                                    // Get the current pose of the right hand
		this->rightHandTrajectory = CartesianTrajectory(T, T, 0.0, endTime);               // Keep the right hand where it is
		
		return true;
	}
	else if(whichHand == "right")
	{
		haiku();
		return true;
	}
	else
	{
		std::cerr << "[ERROR] [HUMANOID] move_to_pose(): String argument was " << whichHand
		          << " but expected 'left' or 'right' as an input." << std::endl;
		return false;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                   Move both hands to a desired pose                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_pose(const iDynTree::Transform &leftHand, const iDynTree::Transform &rightHand)
{
	haiku();
	return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                      Hold the current joint positions                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::halt()
{
	if(isRunning())	stop();                                                                    // Stop any control threads that might be running
	update_state();                                                                            // Get the current joint state
	move_to_position(this->q);                                                                 // Run the control to stay at current configuration	
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     Testing Cartesian force control                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::force_test()
{
	if(isRunning()) stop();                                                                    // Stop any control threads that are running
	this->controlSpace = cartesian;                                                            // Switch to Cartesian control
	start();                                                                                   // Start a new control loop
//	threadInit();
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                              This is executed just after 'start()' is called                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::threadInit()
{
	this->startTime = yarp::os::Time::now();                                                   // Used to regulate the control loop
	return true;
// 	run();
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       MAIN CONTROL LOOP                                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::run()
{
	update_state();                                                                            // Update the joint state information
	double elapsedTime = yarp::os::Time::now() - this->startTime;                              // Get elapsed time since start
	
	iDynTree::VectorDynSize tau(this->n);                                                      // We want to compute this

	switch(this->controlSpace)
	{
		case joint:
		{
			iDynTree::VectorDynSize qddot(this->n);                                    // We want to compute this
			
			// Get the desired joint state
			iDynTree::VectorDynSize q_d(this->n), qdot_d(this->n), qddot_d(this->n);
			this->jointTrajectory.get_state(q_d, qdot_d, qddot_d, elapsedTime);
			
			// Compute the feedforward + feedback control for the joint acceleration
			for(int i = 0; i < this->n; i++)
			{
				qddot[i] = qddot_d[i] + this->Kq*(q_d[i] - this->q[i]) + this->Kd*(qdot_d[i] - this->qdot[i]);
			}
			
//			std::cout << q_d.toString() << std::endl;
			
			// Compute the inverse dynamics from the joint accelerations
			iDynTree::Vector6 baseAcc; baseAcc.zero();                                       // Don't move the base
			iDynTree::LinkNetExternalWrenches wrench(this->model); wrench.zero();            // No external forces applied
			this->computer.inverseDynamics(baseAcc, qddot, wrench, this->generalizedForces); // Solve the inverse dynamics
			tau = this->generalizedForces.jointTorques();                                    // Get the joint torques
			
			break;
		}
		case cartesian:
		{
			// Compute the gravity compensation
			this->computer.generalizedGravityForces(this->generalizedForces);
			tau = this->generalizedForces.jointTorques();
			
			// Variables used in scope
			iDynTree::Transform x_d;                                                   // Desired pose
			iDynTree::Twist xdot_d;                                                    // Desired velocity
			iDynTree::SpatialAcc xddot_d;                                              // Desired acceleration

			print_dynamics();
/*			
			// Get the desired state for the left hand
			std::cout << "So far so good." << std::endl;
			this->leftHandTrajectory.get_state(x_d, xdot_d, xddot_d, elapsedTime);
			std::cout << "\nHere is the desired position of the left hand:" << std::endl;
			std::cout << x_d.getPosition().toString() << std::endl;

			// Compute the gravity compensation
			this->computer.generalizedGravityForces(this->generalizedForces);
			tau = this->generalizedForces.jointTorques();
		
			// Compute the torques from forces at the hand
			Eigen::MatrixXd J(6, 6 + this->n);
			this->computer.getFrameFreeFloatingJacobian("l_hand", J);
			J = J.block(0,6,6,10);                                                     // Just the Jacobian for the hand

			Eigen::VectorXd f(6);
			f << 0, -2, -2, 0, 0, 0;                                                   // Force vector	
			Eigen::VectorXd temp = J.transpose()*f;

			for(int i = 0; i < 10; i++) tau[i] += temp[i];
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

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                           This is executed just after 'stop()' is called                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::threadRelease()
{
	this->computer.generalizedGravityForces(this->generalizedForces);                          // Get the torque needed to withstand gravity
	send_torque_commands(this->generalizedForces.jointTorques());                              // Send to the robot
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                  Get the inverse of a matrix                                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd Humanoid::inverse(const Eigen::MatrixXd &A)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> SVD(A, Eigen::ComputeFullU | Eigen::ComputeFullV);       // Get the SVD decomposition
	Eigen::MatrixXd V = SVD.matrixV();                                                         // V matrix
	Eigen::MatrixXd U = SVD.matrixU();                                                         // U matrix
	Eigen::VectorXd s = SVD.singularValues();                                                  // Get the singular values
	Eigen::MatrixXd invA(A.cols(), A.rows()); invA.setZero();                                  // Value we want to return
	
	for(int i = 0; i < A.cols(); i++)
	{
		for(int j = 0; j < s.size(); j++)
		{
			for(int k = 0; k < A.rows(); k++)
			{
				if(s(j) >= 1e-04)	invA(i,k) += (V(i,j)*U(k,j))/s(j);         // Fast inverse
//				else			invA(i,k) += 0;                            // Ignore singular directions
			}
		}
	}
	return invA;
}
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //			Prints out kinematic values for debugging purposes                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
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

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //			Prints out dynamic values for debugging purposes                          //
///////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::print_dynamics()
{
	// Compute the inertia matrix
	Eigen::MatrixXd M(6+this->n, 6+this->n);
	this->computer.getFreeFloatingMassMatrix(M);
	M = M.block(6,6,10,10);                                                                    // Just the inertia for the left hand
	std::cout << "\nHere is the inertia matrix:" << std::endl;
	std::cout << M << std::endl;
	
	Eigen::MatrixXd invM = inverse(M);                                                         // As it says on the label
	std::cout << "\nHere is the inverse of the inertia matrix:" << std::endl;
	std::cout << invM << std::endl;
	std::cout << "\nHere is the inertia matrix by its inverse:" << std::endl;
	std::cout << M*invM << std::endl;                                                          // Check that it's identity
	
	Eigen::MatrixXd J(6, 6+this->n);                                                           // Jacobian matrix
	this->computer.getFrameFreeFloatingJacobian("l_hand", J);                                  // Get the Jacobian to the left hand
	J = J.block(0,6,6,10);                                                                     // Remove the floating base component
	Eigen::MatrixXd A = inverse(J*invM*J.transpose());                                         // Cartesian inertia matrix of the left hand
	std::cout << "\nHere is the Cartesian inertia matrix:" << std::endl;
	std::cout << A << std::endl;
	
	// Compute the Coriolis and gravity torques
	this->computer.generalizedBiasForces(this->generalizedForces);
	iDynTree::VectorDynSize h = this->generalizedForces.jointTorques();
	std::cout << "\nHere are the Coriolis and gravity torques:" << std::endl;
	std::cout << h.toString() << std::endl;

	// Compute nonlinear acceleration term
	iDynTree::Vector6 Jdotqdot = this->computer.getFrameBiasAcc("l_hand");
	std::cout << "\nHere is the d/dt(J)*dq/dt:" << std::endl;
	std::cout << Jdotqdot.toString() << std::endl;
}

#endif
