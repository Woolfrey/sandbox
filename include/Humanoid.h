    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                     A class for coordinating motion of a humanoid robot                        //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef HUMANOID_H_
#define HUMANOID_H_

#include <CartesianTrajectory.h>                                                                    // Custom class
#include <Eigen/Dense>                                                                              // Matrix inverse and SVD
#include <iDynTree/Core/EigenHelpers.h>                                                             // Converts iDynTree tensors to Eigen
#include <iDynTree/Core/CubicSpline.h>                                                              // For trajectory stuff
#include <iDynTree/KinDynComputations.h>                                                            // Class that does inverse dynamics calculations
#include <iDynTree/Model/FreeFloatingState.h>                                                       // iDynTree::FreeFloatingGeneralizedTorques
#include <iDynTree/Model/Model.h>                                                                   // Class that holds basic kinematic & dynamic info
#include <iDynTree/ModelIO/ModelLoader.h>                                                           // Extracts information from URDF
#include <JointInterface.h>                                                                         // Communicates with motors
#include <Math.h>                                                                                   // Mathematical functions
#include <yarp/os/PeriodicThread.h>                                                                 // Keeps timing of the control loop

std::vector<double> startConfiguration({ 00.00,  00.00,  00.00,                                                              // Torso
					-30.00*M_PI/180,  30.00*M_PI/180,  00.00,  45.00*M_PI/180,  00.00,  00.00,  00.00,   // Left arm
					-30.00*M_PI/180,  30.00*M_PI/180,  00.00,  45.00*M_PI/180,  00.00,  00.00,  00.00}); // Right arm
					
iDynTree::Transform leftHandOffset(iDynTree::Rotation::RPY(0,M_PI/2,0), iDynTree::Position(0.0, 0.0, 0.04));
iDynTree::Transform rightHandOffset(iDynTree::Rotation::RPY(0,M_PI/2,0), iDynTree::Position(0.0, 0.0, 0.04));

std::vector<std::string> jointList = {"torso_pitch","torso_roll","torso_yaw",
				      "l_shoulder_pitch","l_shoulder_roll","l_shoulder_yaw","l_elbow","l_wrist_prosup","l_wrist_pitch","l_wrist_yaw",
				      "r_shoulder_pitch","r_shoulder_roll","r_shoulder_yaw","r_elbow","r_wrist_prosup","r_wrist_pitch","r_wrist_yaw"};
//				      "l_hip_pitch","l_hip_roll","l_hip_yaw","l_knee","l_ankle_pitch","l_ankle_roll",
//				      "r_hip_pitch","r_hip_roll","r_hip_yaw","r_knee","r_ankle_pitch","r_ankle_roll",
//				      "neck_pitch","neck_roll","neck_yaw"};
//				      "neck_fixed_joint"};

class Humanoid : public yarp::os::PeriodicThread,
                 public JointInterface
{
	public:
		Humanoid(const std::string &fileName);
		
		bool move_to_pose(const iDynTree::Transform &desired,                               // Move one hand to a given pose in a given time
		                  const std::string &whichHand,
		                  const double &time);
		
		bool move_to_pose(const iDynTree::Transform &left,                                  // Move both hands to given poses in given time
		                  const iDynTree::Transform &right,
		                  const double &time);
		                  
		bool move_to_poses(const std::vector<iDynTree::Transform> &left,                    // Move both hands through multiple poses
		                   const std::vector<iDynTree::Transform> &right,
		                   const std::vector<double> &time);

		bool move_to_position(const iDynTree::VectorDynSize &position,                      // Move joints to given position in a given time
		                      const double &time);

		bool move_to_positions(const std::vector<iDynTree::VectorDynSize> &positions,       // Move joints through multiple positions
				       const iDynTree::VectorDynSize &times);
				  
		bool translate(const iDynTree::Position &distance,                                  // Translate one hand by the given distance
		               const std::string &whichHand,
		               const double &time);
		
		bool translate(const iDynTree::Position &left,                                      // Translate both hands by the given distance
		               const iDynTree::Position &right,
		               const double &time);

		bool update_state();                                                                // Update the kinematics & dynamics of the robot
		
		void halt();                                                                        // Stop the robot immediately
			               
		// Grasping related functions
		
		bool grasp_object(const iDynTree::Transform &left,
		                  const iDynTree::Transform &right,
		                  const iDynTree::Transform &object);

	private:
		double dt = 0.01;
		
		enum ControlMode {joint, cartesian, grasp} controlMode;
		
		iDynTree::VectorDynSize q, qdot;                                                    // Joint positions and velocities
		
		std::vector<double> minAcc, maxAcc;                                                 // Instantaneous limits on joint acceleration
		
		// Joint control
		double kq = 50.0;
		double kd =  2.0;
		std::vector<iDynTree::CubicSpline> jointTrajectory;
		
		// Cartesian control
		bool leftControl, rightControl;                                                     // Switch for activating left and right control
		CartesianTrajectory leftTrajectory, rightTrajectory;                                // Cartesian trajectories for each hand
		Eigen::Matrix<double,6,6> K, D;                                                     // Cartesian stiffness and damping
		
		// Grasp Control
		CartesianTrajectory objectTrajectory;                                               // As it says on the label
		Eigen::Matrix<double,6,12> G;                                                       // Grasp matrix
		Eigen::Matrix<double,12,6> C;                                                       // Constraint matrix
		Eigen::Matrix<double,6,6> Ao;                                                       // Object inertia
		iDynTree::Transform T_hand_to_object;
		iDynTree::Transform T_object;
		
		// Kinematics & dynamics
		iDynTree::FreeFloatingGeneralizedTorques generalForces;                             // Forces and torques		
		iDynTree::KinDynComputations computer;                                              // Does all the kinematics & dynamics
		iDynTree::Transform torsoPose;                                                      // Needed for inverse dynamics, but not used yet
		iDynTree::Twist torsoTwist;                                                         // Needed for inverse dynamics, but not used yet
		iDynTree::Vector3 gravity;                                                          // Needed for inverse dynamics, but not used yet
	
		// Internal functions
		
		bool limit_joint_acceleration(double &qddot, const int &i);                         // Avoid joint limits
		
		double get_penalty(const int &i);
		
		iDynTree::Vector6 get_pose_error(const iDynTree::Transform &desired,                // Get the pose error between 2 transforms
			                         const iDynTree::Transform &actual);
			                         
		Eigen::VectorXd get_cartesian_control(const iDynTree::Transform &actualPose,
		                                      const iDynTree::Twist &actualVel,
		                                      CartesianTrajectory &trajectory,
		                                      const double &time);
		
		// Control loop functions related to the PeriodicThread class
		double startTime;
		bool threadInit();
		void run();
		void threadRelease();
};                                                                                                  // Semicolon needed after a class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
Humanoid::Humanoid(const std::string &fileName) :
	           yarp::os::PeriodicThread(0.01),                                                  // Create the threading object for control
                   JointInterface(jointList),                                                       // Open communication with the robot
                   torsoPose(iDynTree::Transform(iDynTree::Rotation::RPY(0,0,0),iDynTree::Position(0,0,0.64))),
                   torsoTwist(iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)))
{	
	// Set the gravity vector
	this->gravity(0) =  0.00;
	this->gravity(1) =  0.00;
	this->gravity(2) = -9.81;
	
	// Set the Cartesian control gains	
	this->K <<    1.0,   0.0,   0.0,   0.0,   0.0,   0.0,
		      0.0,   1.0,   0.0,   0.0,   0.0,   0.0,
		      0.0,   0.0,   1.0,   0.0,   0.0,   0.0,
		      0.0,   0.0,   0.0,   1.0,   0.1,   0.0,
		      0.0,   0.0,   0.0,   0.1,   1.0,   0.0,
		      0.0,   0.0,   0.0,   0.0,   0.0,   1.0;
	
	this->K *= 80.0;
		     
	this->D <<    1.0,  0.0,  0.0,  0.0,  0.0,  0.0,
		      0.0,  1.0,  0.0,  0.0,  0.0,  0.0,
		      0.0,  0.0,  1.0,  0.0,  0.0,  0.0,
		      0.0,  0.0,  0.0,  1.0,  0.0,  0.0,
		      0.0,  0.0,  0.0,  0.0,  1.0,  0.0,
		      0.0,  0.0,  0.0,  0.0,  0.0,  1.0;

	this->D *= 15.0;

	// Load a model
	iDynTree::ModelLoader loader;                                                               // Temporary

	if(not loader.loadReducedModelFromFile(fileName, jointList, "urdf"))
	{
		std::cerr << "[ERROR] [HUMANOID] Constructor: "
		          << "Could not load model from path " << fileName << std::endl;
	}
	else
	{
		// Get the model and add some additional frames for the hands
		iDynTree::Model temp = loader.model();
		temp.addAdditionalFrameToLink("l_hand", "left",
					      iDynTree::Transform(iDynTree::Rotation::RPY(0,M_PI/2,0.0),
								  iDynTree::Position(0,0,-0.04)));
		temp.addAdditionalFrameToLink("r_hand", "right",
					      iDynTree::Transform(iDynTree::Rotation::RPY(0,M_PI/2,0.0),
					      			  iDynTree::Position(0,0,-0.04)));
									    
		if(not this->computer.loadRobotModel(temp))
		{
			std::cerr << "[ERROR] [HUMANOID] Constructor: "
				  << "Could not generate iDynTree::KinDynComputations class from given model: "
				  << loader.model().toString() << std::endl;
		}
		else
		{
			this->n = this->computer.model().getNrOfDOFs();                             // Get number of joints from underlying model
			this->generalForces.resize(this->computer.model());                         // Restructure force class to match model

			// Resize vectors based on model information
			this->minAcc.resize(this->n);                                               // State-dependent minimum joint acceleration
			this->maxAcc.resize(this->n);                                               // State-dependent maximum joint acceleration
			this->jointTrajectory.resize(this->n);                                      // Trajectory for joint motion control
			this->q.resize(this->n);                                                    // Vector of joint positions
			this->qdot.resize(this->n);                                                 // Vector of joint velocities

			std::cout << "[INFO] [HUMANOID] Successfully created iDynTree model from " << fileName << "." << std::endl;

			update_state();                                                             // Get the current joint state
			
			if(activate_control()) move_to_position(iDynTree::VectorDynSize(startConfiguration), 3.0);
			else
			{
				std::cerr << "[ERROR] [HUMANOID] Constructor: "
				          << "Could not activate joint control." << std::endl;
			}
		}
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Grasp an object with both hands                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::grasp_object(const iDynTree::Transform &left,
                            const iDynTree::Transform &right,
                            const iDynTree::Transform &object)
{
	// Move the hands to position
	if(not move_to_pose(left, right, 2.0))
	{
		std::cout << "Why isn't this working?!" << std::endl;
		return false;
	}
	else
	{
		while(yarp::os::Time::now() - this->startTime < 2.5);                               // Do nothing while we wait
		
		// Variables used in this scope
		iDynTree::Transform T;                                                              // Temporary placeholder
		iDynTree::Position p;                                                               // Temporary placeholder
		Eigen::MatrixXd Gleft  = Eigen::MatrixXd::Identity(6,6);                            
		Eigen::MatrixXd Gright = Eigen::MatrixXd::Identity(6,6);
		
		// Grasping matrix for the left hand
		T = this->computer.getWorldTransform("left");
		p = object.getPosition() - T.getPosition();
		
		Gleft.block(3,0,3,3) <<   0 , -p(2),  p(1),
		                        p(2),    0 , -p(0),
		                       -p(1),  p(0),    0 ;
		                       
		// Grasping matrix for the right hand
		T = this->computer.getWorldTransform("right");
		p = object.getPosition() - T.getPosition();
		
		Gright.block(3,0,3,3) <<  0 , -p(2),  p(1),
		                        p(2),    0 , -p(0),
		                       -p(1),  p(0),    0 ;
		                       
		// Set new grasping matrices
		this->G.block(0,0,6,6) = Gleft;
		this->G.block(0,6,6,6) = Gright;
		
		this->C.block(0,0,6,6) = Gleft.inverse();
		this->C.block(6,0,6,6) =-Gright.inverse();

		// Set new object frame relative to left hand
		// NOTE: iDynTree::KinDynComputations class will not let me set a new transform since
		// the subsequent iDynTree::Model class is constant (ノಠ益ಠ)ノ彡┻━┻
		this->T_hand_to_object = this->computer.getWorldTransform("left").inverse()*object;
		

		// Set the object inertia
		Ao <<  0.1, 0.0, 0.0, 0.0000, 0.0000, 0.0000,
		       0.0, 0.1, 0.0, 0.0000, 0.0000, 0.0000,
		       0.0, 0.0, 0.1, 0.0000, 0.0000, 0.0000,
		       0.0, 0.0, 0.0, 0.0001, 0.0000, 0.0000,
		       0.0, 0.0, 0.0, 0.0000, 0.0001, 0.0000,
		       0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0001;
									    
//		this->controlMode = grasp;
		start();
//		jump to threadInit();
					
		return true;
	}

}


///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Move a single hand to a desired pose                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_pose(const iDynTree::Transform &desired,
                            const std::string &whichHand,
                            const double &time)
{	
	if(not(whichHand == "left") and not(whichHand == "right"))
	{
		std::cerr << "[ERROR] [HUMANOID] move_to_pose(): "
			  << "Expected 'left or 'right' as an argument, "
			  << "but your input was " << whichHand << "." << std::endl;
			  
		return false;
	}
	else
	{
		if(isRunning()) stop();                                                             // Stop any control threads that are running
		this->controlMode = cartesian;                                                      // Switch to Cartesian control mode
		
		iDynTree::VectorDynSize times(2); times(0) = 0.0; times(1) = time;                  // Set up times for the trajectory generator
		
		std::vector<iDynTree::Transform> waypoints;                                         // Waypoint to pass to trajectory generator
		waypoints.push_back( this->computer.getWorldTransform(whichHand) );                 // Start pose is current hand location
		waypoints.push_back( desired );                                                     // End pose is desired hand location
		
		if(whichHand == "left")
		{
			this->leftControl = true;
			this->rightControl = false;
			
			this->leftTrajectory = CartesianTrajectory(waypoints, times);
		}
		else
		{
			this->leftControl = false;
			this->rightControl = true;
			
			this->rightTrajectory = CartesianTrajectory(waypoints, times);
		}
		
		start();
//              threadInit(); this is executed immediately after start() is called
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Move both hands to a desired pose                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_pose(const iDynTree::Transform &left,
			    const iDynTree::Transform &right,
			    const double &time)
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	this->controlMode = cartesian;                                                             // Switch to Cartesian control mode
	
	this->leftControl = true; this->rightControl = true;                                        // Activate control of both hands
	
	iDynTree::VectorDynSize times(2); times[0] = 0.0; times[1] = time;                          // Set up times for the trajectory
	
	std::vector<iDynTree::Transform> waypoint; waypoint.resize(2);
	
	// Set up left hand trajectory
	waypoint[0] = this->computer.getWorldTransform("left");                                     // Current left hand pose
	waypoint[1] = left;                                                                         // Desired left hand pose
	this->leftTrajectory = CartesianTrajectory(waypoint, times);                                // Create new trajectory for left hand
	
	// Set up right hand trajectory
	waypoint[0] = this->computer.getWorldTransform("right");                                    // Current right hand pose
	waypoint[1] = right;                                                                        // Desired right hand pose
	this->rightTrajectory = CartesianTrajectory(waypoint,times);                                // Create new trajectory for right hand
	
	start();
//      jump to threadInit();
	return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //          Move the joints to a desired configuration (optimal time solved automatically)       //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_position(const iDynTree::VectorDynSize &position,
                                const double &time)
{
	if(position.size() != this->n)
	{
		std::cerr << "[ERROR] [HUMANOID] move_to_position(): "
			  << "Position vector had " << position.size() << " elements, "
			  << "but this model has " << this->n << " joints." << std::endl;
			  
		return false;
	}
	else
	{
		std::vector<iDynTree::VectorDynSize> target; target.push_back(position);            // Create an array of 1      
		iDynTree::VectorDynSize times(1); times[0] = time;                                  // Time in which to reach target position
		
		return move_to_positions(target,times);
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Move the joints to several desired configurations at given time               //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_positions(const std::vector<iDynTree::VectorDynSize> &positions,
				 const iDynTree::VectorDynSize &times)
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	this->controlMode = joint;                                                                 // Switch to joint control
	
	bool allGood;
	int m = positions.size() + 1;                                                               // 1 extra waypoint for starting position
	iDynTree::VectorDynSize waypoint(m);                                                        // All waypoints of a single joint
	iDynTree::VectorDynSize t(m);                                                               // Times to reach each waypoint
	for(int i = 0; i < this->n; i++)                                                            // ... ith joint
	{
		for(int j = 0; j < m; j++)                                                          // ... jth waypoint
		{
			if(j == 0)
			{
				waypoint[j] = this->q[i];                                           // Use current joint value
				t[j] = 0.0;                                                         // Start immediately
			}
			else
			{
				double target = positions[j-1][i];                                  // jth target of ith joint
				if(target < this->pMin[i])      target = this->pMin[i] + 0.001;     // Just above the lower limit
				else if(target > this->pMax[i]) target = this->pMax[i] - 0.001;     // Just below the upper limit
				waypoint[j] = target;                                               // Assign the target
				t[j] = times[j-1];                                                  // Add on subsequent time data
			}
		}
		allGood = this->jointTrajectory[i].setData(t,waypoint);                             // Set new trajectory data
	}
	
	if(allGood)
	{
		start();
		return true;
	}
	else 	return false;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Translate a hand by the specified amount                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::translate(const iDynTree::Position &distance,
                         const std::string &whichHand,
                         const double &time)
{
	if(whichHand != "left" and whichHand != "right")
	{
		std::cerr << "[ERROR] [HUMANOID] translate(): "
			  << "Expected 'left' or 'right' as an argument, "
			  << "but your input was " << whichHand << "." << std::endl;
			  
		return false;
	}
	else
	{
		iDynTree::Transform T1 = this->computer.getWorldTransform(whichHand);               // Get current pose of the hand

		iDynTree::Transform T2(T1.getRotation(), T1.getPosition() + distance);              // Offset the translation by the given distance

		return move_to_pose(T2, whichHand, time);                                           // Move the hand to the given pose
	}
}
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Translate both hands by the specified amount                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::translate(const iDynTree::Position &leftDistance,
                         const iDynTree::Position &rightDistance,
                         const double &time)
{
	iDynTree::Transform leftTF = this->computer.getWorldTransform("left");
	iDynTree::Transform T1(leftTF.getRotation(), leftTF.getPosition() + leftDistance);
	
	iDynTree::Transform rightTF = this->computer.getWorldTransform("right");
	iDynTree::Transform T2(rightTF.getRotation(), rightTF.getPosition() + rightDistance);
	
	return move_to_pose(T1, T2, time);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Update the joint state for all the limbs                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::update_state()
{
	if(JointInterface::read_encoders())
	{
		get_joint_state(this->q, this->qdot);                                               // Get state as iDynTree objects from interface class
		
		if(this->computer.setRobotState(this->torsoPose,
		                                this->q,
		                                this->torsoTwist,
		                                this->qdot,
		                                this->gravity))
		{			
			return true;
		}
		else
		{
			std::cerr << "[ERROR] [HUMANOID] update_state(): "
				  << "Could not set state for the iDynTree::iKinDynComputations object." << std::endl;
				  
			return false;
		}
	}
	else
	{
		std::cerr << "[ERROR] [HUMANOID] update_state(): "
			  << "Could not update state from the JointInterface class." << std::endl;
			  
		return false;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Hold the current joint positions                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::halt()
{
	if(isRunning())	stop();                                                                     // Stop any control threads that might be running
	move_to_position(this->q, 2.0);                                                             // Run control to stay at current configuration
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                              This is executed just after 'start()' is called                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::threadInit()
{
	this->startTime = yarp::os::Time::now();                                                    // Used to regulate the control loop
	return true;
// 	jump to run();
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       MAIN CONTROL LOOP                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::run()
{
	update_state();                                                                             // Get new joint state
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;                               // Elapsed time since start of control loop
	
	iDynTree::VectorDynSize tau(this->n);                                                       // We want to compute this
	
	if(this->controlMode == joint)
	{
		iDynTree::VectorDynSize ctrl(this->n),
			                qddot_d(this->n),
			                qdot_d(this->n),
			                q_d(this->n);
		
		// Solve the PD control for each joint               
		for(int i = 0; i < this->n; i++)
		{
			q_d[i] = this->jointTrajectory[i].evaluatePoint(elapsedTime,                // Get the desired state
				                                        qdot_d[i],
				                                        qddot_d[i]);
			
			ctrl[i] = qddot_d[i]                                                        // Feedforward term
			        + this->kd*(qdot_d[i] - this->qdot[i])                              // Velocity feedback
			        + this->kq*(q_d[i] - this->q[i]);                                   // Position feedback
			        
			
			if(limit_joint_acceleration(ctrl[i], i))
			{
				std::cout << "[WARNING] [HUMANOID] "
				          << jointList[i] << " joint hit a limit." << std::endl;
			}
		}
			
		// Compute the inverse dynamics from the joint acceleration
		iDynTree::Vector6 baseAcc; baseAcc.zero();                                          // Zero base acceleration
		iDynTree::LinkNetExternalWrenches wrench(this->computer.model()); wrench.zero();    // No external wrenches
		this->computer.inverseDynamics(baseAcc, ctrl, wrench, this->generalForces);         // Solve the inverse dynamics
		
		tau = this->generalForces.jointTorques();                                           // Extract the joint torque vector
	}
	else
	{
		// Generate the Jacobian
		Eigen::MatrixXd J(12,this->n);                                                      // Jacobian for both hands
		Eigen::MatrixXd temp(6,6+this->n);                                                  // Jacobian for a single hand
		
		this->computer.getFrameFreeFloatingJacobian("left", temp);                          // Get the full left hand Jacobian
		J.block(0,0,6,this->n) = temp.block(0,6,6,this->n);                                 // Assign to the larger Jacobian
		if(not this->leftControl) J.block(0,0,6,3).setZero();                               // Remove contribution of torso joints
		
		this->computer.getFrameFreeFloatingJacobian("right", temp);                         // Get the full right hand Jacobian
		J.block(6,0,6,this->n) = temp.block(0,6,6,this->n);                                 // Assign the right hand Jacobian
		if(not this->rightControl) J.block(6,0,6,3).setZero();                              // Remove contribution of torso joints
		
		Eigen::MatrixXd Jt = J.transpose();                                                 // Makes calcs a little easier later
		
		// Compute the joint and Cartesian inertia matrices
		Eigen::MatrixXd M(6+this->n,6+this->n);                                             // Storage location
		this->computer.getFreeFloatingMassMatrix(M);                                        // Inertia including floating base
		M = M.block(6,6,this->n,this->n);                                                   // Remove the floating base part
		Eigen::MatrixXd invM = get_inverse(M);                                              // Invert the inertia matrix
		Eigen::MatrixXd A = get_inverse(J*invM*Jt);                                         // Cartesian inertia matrix
		
		// Compute the bias accelerations & forces
		Eigen::VectorXd accBias(12);                                                        // (dJ/dt)*(dq/dt)
		accBias.block(0,0,6,1) = iDynTree::toEigen(this->computer.getFrameBiasAcc("left"));
		accBias.block(6,0,6,1) = iDynTree::toEigen(this->computer.getFrameBiasAcc("right"));
		
		this->computer.generalizedBiasForces(this->generalForces);			
		Eigen::VectorXd coriolisAndGravity = iDynTree::toEigen(this->generalForces.jointTorques());
		
		// Compute the null space torques
		// NOTE: DAMPING TERM CAN CAUSE INSTABILITY IF TOO HIGH
		Eigen::VectorXd qddot_d(this->n);
		for(int i = 0; i < this->n; i++) qddot_d[i] = -2*this->qdot[i] - get_penalty(i);
		
		Eigen::VectorXd qddot = Eigen::VectorXd::Zero(this->n);
		
		if(controlMode == cartesian)
		{
			// Compute the Cartesian control
			Eigen::VectorXd xddot = Eigen::VectorXd::Zero(12);

			if(this->leftControl) xddot.block(0,0,6,1) = get_cartesian_control(this->computer.getWorldTransform("left"),
				                                                           this->computer.getFrameVel("left"),
				                                                           this->leftTrajectory,
				                                                           elapsedTime);
				                                                          
			if(this->rightControl) xddot.block(6,0,6,1) = get_cartesian_control(this->computer.getWorldTransform("right"),
				                                                            this->computer.getFrameVel("right"),
				                                                            this->rightTrajectory,
				                                                            elapsedTime);
			
			// NOTE TO SELF:
			// This is not quite correct. xddot = xddot_d - Jdot*qdot + inv(A)*(De + Ke) for it to be correct...
			qddot = invM*Jt*A*xddot
		              + (Eigen::MatrixXd::Identity(this->n,this->n) - invM*Jt*A*J)*qddot_d;
			
			for(int i = 0; i < this->n; i++)
			{
				if(limit_joint_acceleration(qddot(i),i))
				{
	//				std::cout << "[WARNING] [HUMANOID] "
	//				          << jointList[i] << " joint hit a joint limit!" << std::endl;
				}
			}
		}
		else if(controlMode == grasp)
		{
			Eigen::VectorXd fc(6); fc << 00.0, 10.0, 00.0, 00.0, 00.0, 00.0;
			
			std::cout << "\nHere are the grasp forces:" << std::endl;
			std::cout << C*fc << std::endl;
		}
		
		Eigen::VectorXd torque = M*qddot;
		
		for(int i = 0; i < this->n; i++) tau[i] = torque[i] + coriolisAndGravity[i];
	}
	
	send_torque_commands(tau);                                                                  // Send the commands to the joint motors
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                           This is executed just after 'stop()' is called                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::threadRelease()
{
	this->computer.generalizedGravityForces(this->generalForces);                               // Get the torque needed to withstand gravity
	send_torque_commands(this->generalForces.jointTorques());                                   // Send to the robot
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Solve the Cartesian motion to track a trajectory                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd Humanoid::get_cartesian_control(const iDynTree::Transform &actualPose,
                                                const iDynTree::Twist &actualVel,
                                                      CartesianTrajectory &trajectory,
                                                const double &time)
{
	// Solve for the desired state
	iDynTree::Transform  desiredPose;
	iDynTree::Twist      vel;
	iDynTree::SpatialAcc acc;
	
	trajectory.get_state(desiredPose, vel, acc, time);
	
	// Compute the state error
	Eigen::VectorXd e = iDynTree::toEigen(get_pose_error(desiredPose, actualPose));
	Eigen::VectorXd edot = iDynTree::toEigen(vel - actualVel);
	
	return iDynTree::toEigen(acc) + this->D*edot + this->K*e;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                      Compute pose error                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
iDynTree::Vector6 Humanoid::get_pose_error(const iDynTree::Transform &desired,
                                           const iDynTree::Transform &actual)
{
	iDynTree::Position pos = desired.getPosition() - actual.getPosition();
	
	iDynTree::Vector4 quat = (desired.getRotation()*actual.getRotation().inverse()).asQuaternion(); // Get the rotation error as a quaternion
	
	double angle = 2*acos(quat(0));                                                             // Orientation error as a single angle
	
	iDynTree::Vector6 error;                                                                    // Value to be returned
	for(int i = 0; i < 3; i++)
	{
		error(i) = pos(i);                                                                  // Transfer pose error
		
		if(angle < M_PI) error(i+3) =  quat(i+1);                                           // Orientation error = quat vector component
		else             error(i+3) = -quat(i+1);                                           // Angle > 180 degrees, so go other direction
	}

	return error;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //	                               Limit joint acceleration                                   //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::limit_joint_acceleration(double &qddot, const int &i)
{
	double min = std::max(2*(this->pMin[i] - this->q[i] - this->dt*this->qdot[i])/(this->dt*this->dt), // Position limit
	             std::max((-this->vMax[i] - this->qdot[i])/this->dt,                                  // Velocity limit
	                       -10.0));                                                                   // Acceleration limit
	
	double max = std::min(2*(this->pMax[i] - this->q[i] - this->dt*this->qdot[i])/(this->dt*this->dt),
	             std::min(( this->vMax[i] - this->qdot[i])/this->dt,
	                        10.0));
	if(qddot < min)
	{
		qddot = min + 0.0001;;
		return true;
	}
	else if(qddot > max)
	{
		qddot = max - 0.0001;
		return true;
	}
	else    return false;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //	                 Get the penalty function for joint limit avoidance                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
double Humanoid::get_penalty(const int &i)
{
	double upper = this->pMax[i] - this->q(i);                                                  // Distance to upper limit
	double lower = this->q(i) - this->pMin[i];                                                  // Distance from lower limit
	double range = this->pMax[i] - this->pMin[i];                                               // Distance between limits
	
//      double penalty = range*range/(4*upper*lower);                                               // Penalty function, but only need derivative

	double dpdq = (range*range*(2*this->q(i) - this->pMax[i] - this->pMin[i]))                  // Derivative (i.e. gradient)
	             /(4*upper*upper*lower*lower);

	     if(dpdq >  1e02) dpdq =  1e02;
	else if(dpdq < -1e02) dpdq = -1e02;
	
	return dpdq;
}

#endif
