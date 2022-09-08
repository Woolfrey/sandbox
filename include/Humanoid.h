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
#include <QPSolver.h>
#include <yarp/os/PeriodicThread.h>                                                                 // Keeps timing of the control loop

std::vector<double> startConfiguration({ 00.00,  00.00,  00.00,                                                              // Torso
					-30.00*M_PI/180,  30.00*M_PI/180,  00.00,  45.00*M_PI/180, -45.00*M_PI/180,  00.00,  00.00,   // Left arm
					-30.00*M_PI/180,  30.00*M_PI/180,  00.00,  45.00*M_PI/180, -45.00*M_PI/180,  00.00,  00.00}); // Right arm
					
						
					
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
                 public JointInterface,
                 public QPSolver
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
		                  const iDynTree::Transform &right);

	private:
		double dt = 0.01;
		
		enum ControlMode {joint, cartesian, grasp} controlMode;
		
		iDynTree::VectorDynSize q, qdot, qddot, tau;                                        // Joint state
		
		// Joint control
		double kq = 50.0;
		double kd = 15.0;
		std::vector<iDynTree::CubicSpline> jointTrajectory;
		
		// Cartesian control
		bool leftControl, rightControl;                                                     // Switch for activating left and right control
		CartesianTrajectory leftTrajectory, rightTrajectory;                                // Individual trajectories for left, right hand
		Eigen::Matrix<double,6,6> K, D;                                                     // Cartesian stiffness and damping
		
		// Grasp control
		CartesianTrajectory objectTrajectory;
		Eigen::Matrix<double, 6, 6> Ao;
		Eigen::Matrix<double,12, 6> C;                                                      // Constraint matrix
		Eigen::Matrix<double, 6,12> G;                                                      // Grasp matrix
		Eigen::Matrix<double,24, 6> Bc;                                                     // Constraint matrix for QP solver
		Eigen::Matrix<double,24, 1> zc;                                                     // Constraint vector for QP solver
		Eigen::Matrix<double,12, 1> fMin, fMax;                                             // Maximum and minimum grasping forces
		
		
		// Kinematics & dynamics
		iDynTree::FreeFloatingGeneralizedTorques generalForces;                             // Forces and torques		
		iDynTree::KinDynComputations             computer;                                  // Does all the kinematics & dynamics
		iDynTree::Transform                      torsoPose;                                 // Needed for inverse dynamics, but not used yet
		iDynTree::Twist                          torsoTwist;                                // Needed for inverse dynamics, but not used yet
		iDynTree::Vector3                        gravity;                                   // Needed for inverse dynamics, but not used yet
		
	
		// Internal functions
		
		bool get_acceleration_limits(double &lower, double &upper, const int &jointNum);
		
		double get_joint_penalty(const int &jointNum);
		
		iDynTree::Vector6 get_pose_error(const iDynTree::Transform &desired,                // Get the pose error between 2 transforms
			                         const iDynTree::Transform &actual);
			                       
		Eigen::VectorXd get_feedforward_feedback(Eigen::VectorXd           &feedback,
			                                 CartesianTrajectory       &trajectory,
		                                         const iDynTree::Transform &actualPose,
		                                         const iDynTree::Twist     &actualVel,
		                                         const double              &time);
		
		// Control loop functions related to the PeriodicThread class
		double startTime;
		virtual bool threadInit();
		virtual void run();
		virtual void threadRelease();
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

	this->D *= 1.0;

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
			this->jointTrajectory.resize(this->n);                                      // Trajectory for joint motion control
			this->q.resize(this->n);                                                    // Vector of joint positions
			this->qdot.resize(this->n);                                                 // Vector of joint velocities
			this->qddot.resize(this->n);                                                // Vector of joint accelerations
			this->tau.resize(this->n);                                                  // Vector of joint torques

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
 //                             Move a single hand to a desired pose                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_pose(const iDynTree::Transform &desired,
                            const std::string         &whichHand,
                            const double              &time)
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
			    const double              &time)
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
 //                          Move both hands through multiple poses                               //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_poses(const std::vector<iDynTree::Transform> &left,
                             const std::vector<iDynTree::Transform> &right,
                             const std::vector<double> &time)
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	this->controlMode = cartesian;                                                              // Activate Cartesian control
	this->leftControl = true; this->rightControl = true;                                        // Activate both hands
	
	// Set up the time trajectory
	iDynTree::VectorDynSize times(time.size() + 1);
	times[0] = 0.0;
	for(int i = 1; i < times.size(); i++) times[i] = time[i-1];
	
	// Set up the left hand trajectory
	std::vector<iDynTree::Transform> waypoint;                                                  // We will have 1 additional waypoint
	waypoint.push_back(this->computer.getWorldTransform("left"));
	waypoint.insert(waypoint.end(),left.begin(),left.end());                                    // Add other waypoints to the end
	this->leftTrajectory = CartesianTrajectory(waypoint, times);
	
	// Set up the right hand trajectory
	waypoint.clear();
	waypoint.push_back(this->computer.getWorldTransform("right"));
	waypoint.insert(waypoint.end(),right.begin(),right.end());
	this->rightTrajectory = CartesianTrajectory(waypoint, times);
	
	start(); // Jump immediately to threadInit();
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
	this->controlMode = joint;                                                                  // Switch to joint control
	
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
                         const std::string        &whichHand,
                         const double             &time)
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
                         const double             &time)
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
//		get_joint_state(this->q, this->qdot);                                               // Get state as iDynTree objects
		get_joint_state(this->q, this->qdot, this->qddot, this->tau);
		
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
	if(isRunning())	stop();                                                                     // Stop any control threads running
	move_to_position(this->q, 2.0);                                                             // Run control to stay at current configuration
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                      Grasp an object                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::grasp_object(const iDynTree::Transform &left, const iDynTree::Transform &right)
{
	if(isRunning()) stop();                                                                     // Stop any control threads that are running
	
	// Set up spline to move hands to side of box
	std::vector<iDynTree::Transform> leftWaypoints;                                             // Start at current hand pose
	leftWaypoints.push_back(iDynTree::Transform(left.getRotation(),
	                                            left.getPosition() + iDynTree::Position(0,0.05,0))); // Set waypoint off the side
	leftWaypoints.push_back(left);                                                              // Finish at desired pose
	
	std::vector<iDynTree::Transform> rightWaypoints;
	rightWaypoints.push_back(iDynTree::Transform(right.getRotation(),
	                                             right.getPosition() - iDynTree::Position(0,0.05,0)));
	rightWaypoints.push_back(right);
	
	// Set up waypoint times
	std::vector<double> times;
	times.push_back(2.5);
	times.push_back(3.0);
	
	move_to_poses(leftWaypoints, rightWaypoints,times);
	
	// Wait for the action to be completed
	float tick = yarp::os::Time::now();
	do { /* nothing */}
	while (yarp::os::Time::now() - tick <= times.back());
	
	// Set up the grasp and constraint matrices
	iDynTree::Transform leftTransform  = this->computer.getWorldTransform("left");
	iDynTree::Transform rightTransform = this->computer.getWorldTransform("right");
	
	iDynTree::Position midPoint = leftTransform.getPosition() + rightTransform.getPosition();
	for(int i = 0; i < 3; i++) midPoint(i) *= 0.5;
	
	iDynTree::Position p = leftTransform.getPosition() - midPoint;
	
	// Left grasp matrix
	Eigen::MatrixXd Gleft = Eigen::MatrixXd::Identity(6,6);
	Gleft.block(3,0,3,3) <<    0, -p(2),  p(1),
	                        p(2),     0, -p(0),
	                       -p(1),  p(0),     0;
	                 
	// Right grasp matrix      
	p = rightTransform.getPosition() - midPoint;
	Eigen::MatrixXd Gright = Eigen::MatrixXd::Identity(6,6);
	Gright.block(3,0,3,3) <<    0, -p(2),  p(1),
	                         p(2),     0, -p(0),
	                        -p(1),  p(0),     0;
	
	// Composite grasp matrix
	Eigen::MatrixXd G(6,12);
	G.block(0,0,6,6) = Gleft;
	G.block(0,6,6,6) = Gright;
	
	// Constraint matrix for hands
	this->C.block(0,0,6,6) = Gleft.inverse();
	this->C.block(6,0,6,6) =-Gright.inverse();
	
	// Constraint for QP solver
	this->Bc.block( 0,0,12,6) =-this->C;
	this->Bc.block(12,0,12,6) = this->C;
	
	// Constraint vector for QP solver
	// Left hand:
	this->fMin.block(0,0,6,1) << -10, -10, -10, -5, -5, -5;
	this->fMax.block(0,0,6,1) <<  10,  -5,  10,  5,  5,  5;
	
	// Right hand:
	this->fMin.block(6,0,6,1) << -10,   5, -10, -5, -5, -5;
	this->fMax.block(6,0,6,1) <<  10,  10,  10,  5,  5,  5;
	
	this->zc.head(12) = -this->fMax;
	this->zc.tail(12) =  this->fMin;
	
	// Mass-inertia matrix of the object
	this->Ao.block(0,0,3,3) = 0.1*Eigen::MatrixXd::Identity(3,3);
	this->Ao.block(3,3,3,3) << 1E-3,  0.0,  0.0, 
	                            0.0, 1E-3,  0.0,
	                            0.0,  0.0, 1E-3;
	return true;
	
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
	
	iDynTree::VectorDynSize controlInput(this->n);                                              // To be computed
	
	if(this->controlMode == joint)
	{
		iDynTree::VectorDynSize refAcc(this->n),
			                qddot_d(this->n),
			                qdot_d(this->n),
			                q_d(this->n);
		
		// Solve the PD control for each joint               
		for(int i = 0; i < this->n; i++)
		{
			q_d[i] = this->jointTrajectory[i].evaluatePoint(elapsedTime,                // Get the desired state
				                                        qdot_d[i],
				                                        qddot_d[i]);
			
			refAcc[i] = qddot_d[i]                                                      // Feedforward term
			          + this->kd*(qdot_d[i] - this->qdot[i])                            // Velocity feedback
			          + this->kq*(q_d[i] - this->q[i]);                                 // Position feedback
			        
		}
			
		// Compute the inverse dynamics from the joint acceleration
		iDynTree::Vector6 baseAcc; baseAcc.zero();                                          // Zero base acceleration
		iDynTree::LinkNetExternalWrenches wrench(this->computer.model()); wrench.zero();    // No external wrenches
		this->computer.inverseDynamics(baseAcc, refAcc, wrench, this->generalForces);       // Solve the inverse dynamics
		
		controlInput = this->generalForces.jointTorques();                                  // Extract the joint torque vector
	}
	else
	{
		//////////////////// Compute properties for all Cartesian control //////////////////
		
		// Generate the Jacobian
		Eigen::MatrixXd J(12,this->n);                                                      // Jacobian for both hands
		Eigen::MatrixXd tempJ(6,6+this->n);                                                 // Jacobian for a single hand
		
		this->computer.getFrameFreeFloatingJacobian("left", tempJ);                         // Get the full left hand Jacobian
		J.block(0,0,6,this->n) = tempJ.block(0,6,6,this->n);                                // Assign to the larger Jacobian
		if(not this->leftControl) J.block(0,0,6,3).setZero();                               // Remove contribution of torso joints
		
		this->computer.getFrameFreeFloatingJacobian("right", tempJ);                        // Get the full right hand Jacobian
		J.block(6,0,6,this->n) = tempJ.block(0,6,6,this->n);                                // Assign the right hand Jacobian
		if(not this->rightControl) J.block(6,0,6,3).setZero();                              // Remove contribution of torso joints
		
		Eigen::MatrixXd Jt = J.transpose();                                                 // Makes calcs a little easier later
		
		// Compute the joint and Cartesian inertia matrices
		Eigen::MatrixXd M(6+this->n,6+this->n);                                             // Storage location
		this->computer.getFreeFloatingMassMatrix(M);                                        // Inertia including floating base
		M = M.block(6,6,this->n,this->n);                                                   // Remove the floating base part
		
		Eigen::MatrixXd A = get_inverse(J*get_inverse(M)*Jt);                               // Cartesian inertia matrix (not ideal)
		
		// Compute the nonlinear accelerations and torques
		Eigen::VectorXd accBias(12);                                                        // (dJ/dt)*(dq/dt)
		accBias.block(0,0,6,1) = iDynTree::toEigen(this->computer.getFrameBiasAcc("left"));
		accBias.block(6,0,6,1) = iDynTree::toEigen(this->computer.getFrameBiasAcc("right"));
	
		this->computer.generalizedBiasForces(this->generalForces);			    // C*qdot + g
		Eigen::VectorXd coriolisAndGravity = iDynTree::toEigen(this->generalForces.jointTorques());
		
		
		////////////////////// Compute the relevant hand acceleration //////////////////////
		
		Eigen::Matrix<double,12,1> xddot;                                                   // Value to be computed
		
		if(this->controlMode == cartesian)
		{
			Eigen::VectorXd temp(6);
			Eigen::VectorXd feedback(12);
			
			if(this->leftControl)
			{
				xddot.head(6) = get_feedforward_feedback(temp,                              
				                                         this->leftTrajectory,
				                                         this->computer.getWorldTransform("left"),
				                                         this->computer.getFrameVel("left"),
				                                         elapsedTime);
				feedback.head(6) = temp;                                            // Assign feedback component to vector
			}
			else	feedback.head(6) = -2*J.block(0,0,6,this->n)*iDynTree::toEigen(this->qdot); // Slow down the hand
			
			if(this->rightControl)
			{
				xddot.tail(6) = get_feedforward_feedback(temp,
					                                 this->rightTrajectory,
					                                 this->computer.getWorldTransform("right"),
					                                 this->computer.getFrameVel("right"),
					                                 elapsedTime);
				feedback.tail(6) = temp;
			}
			else    feedback.tail(6) = -2*J.block(6,0,6,this->n)*iDynTree::toEigen(this->qdot); // Slow down the hand

			// NOTE: llt().solve() fails when inertia is ill-conditioned
			xddot += J*M.partialPivLu().solve(J.transpose()*feedback);                  // Resolve the dynamic coupling between arms
		}
		else // this->controlMode = grasp
		{
			Eigen::VectorXd feedback(6);
			Eigen::VectorXd xddot = get_feedforward_feedback(feedback,
			                                                 this->objectTrajectory,
			                                                 this->computer.getWorldTransform("object"),
			                                                 this->computer.getFrameVel("object"),
			                                                 elapsedTime);
			
			xddot(2) += 9.81;                                                           // Need to accelerate up with gravity
			Eigen::VectorXd force = this->Ao*xddot + feedback;                          // Sum all forces on object
			
			// Resolve dynamic coupling between object and arms
			xddot = this->G.transpose()*((this->Ao + this->G*A*this->G.transpose()).partialPivLu().solve(force));
		}
		
		//////////////////////////// Solve the Cartesian force control /////////////////////
		
		// Variables used for QP solver: min 0.5*x'*H*x subject to: B*z >= z
		Eigen::MatrixXd H(12+this->n,12+this->n);                                           // Hessian for the QP solver
		Eigen::VectorXd f(12+this->n);                                                      // Constant force the QP solver
		Eigen::MatrixXd B(4*this->n,12+this->n);                                            // Constraint matrix for QP solver
		Eigen::VectorXd z(4*this->n);                                                       // Constraint vector
		Eigen::VectorXd redundant(this->n);
		Eigen::VectorXd initialGuess(12+this->n); initialGuess.head(12) = A*(xddot - accBias);
		
		// H = [ 0   J ]
		//     [ J'  M ]
		H.block( 0, 0,     12,     12) = Eigen::MatrixXd::Zero(12,12);
		H.block( 0,12,     12,this->n) = J;
		H.block(12, 0,this->n,     12) = J.transpose();
		H.block(12,12,this->n,this->n) = M;
		
		// f = [ xddot - Jdot*qdot ]
		//     [    redundant    ]
		f.head(12) = accBias - xddot;                                                       // Need opposite sign for the QP solver
//		f.tail(this->n) = -redundant; // NOTE: Redundant task computed in for-loop below

		// B = [ 0 -I ]
		//     [ 0  I ]
		//     [ 0 -M ]
		//     [ 0  M ]
		B.block(        0, 0,4*this->n,     12) =  Eigen::MatrixXd::Zero(4*this->n,12);
		B.block(        0,12,  this->n,this->n) = -Eigen::MatrixXd::Identity(this->n,this->n);
		B.block(  this->n,12,  this->n,this->n) =  Eigen::MatrixXd::Identity(this->n,this->n);
		B.block(2*this->n,12,  this->n,this->n) = -M;
		B.block(3*this->n,12,  this->n,this->n) =  M;
		
		// z = [ -maxAcc ]
		//     [  minAcc ]
		//     [ -maxTau ]
		//     [  minTau ]		
		for(int i = 0; i < this->n; i++)
	        {
	        	double lower, upper;
	        	get_acceleration_limits(lower, upper, i);
	        	z(i)           = -upper;                                                    // Maximum acceleration
	        	z(i+1*this->n) =  lower;                                                    // Minimum acceleration
	        	z(i+2*this->n) = -(this->tMax[i] - coriolisAndGravity(i));                  // -M*qddot >= -(tau - h)
	        	z(i+3*this->n) = -this->tMax[i] - coriolisAndGravity(i);                    // Minimum torque
   
			redundant(i) = -2*this->qdot(i);
	        	
	        	initialGuess(12+i) = 0.5*(lower+upper);                                     // Halfway?
	        }
	        f.tail(this->n) = -redundant;                                                       // We need opposite sign for the QP solver
	        
	        Eigen::VectorXd qddot = (solve(H,f,B,z,initialGuess)).tail(this->n);                // Find optimal joint accelerations
	        
	        Eigen::VectorXd torque = M*qddot;                                                   // Convert to joint torques
		
		///////////////////// Project in to null space of grasping ////////////////////////
		if(this->controlMode == grasp)
		{
			Eigen::MatrixXd Jc  = this->C.transpose()*J;
			Eigen::MatrixXd Jct = Jc.transpose();
			
			//////////////////////// Solve the constraint forces ///////////////////////
			Eigen::VectorXd fc_0(6); fc_0 << 0, 0, -6, 0, 0, 0;                         // Initial guess for forces
			Eigen::VectorXd fc = solve(Jc*Jct,Eigen::VectorXd::Zero(6),Bc,zc,fc_0);     // Optimise the constraint forces
			
			/////////////////////  Solve for the projected control /////////////////////
			H.resize(6+this->n,6+this->n);
			f.resize(6+this->n);
			initialGuess.resize(6+this->n);
			
			// H = [ 0   Jc ]
			//     [ Jc' M  ]
			H.block(0,0,      6,      6) = Eigen::MatrixXd::Zero(6,6);
			H.block(0,6,      6,this->n) = Jc;
			H.block(6,0,this->n,      6) = Jct;
			H.block(6,6,this->n,this->n) = M;
			
			// f = [ Jcdot*qdot ]
			//     [   -tau_n   ]
			f.head(6)       = this->C.transpose()*accBias;                              // Jcdot*qdot
			f.tail(this->n) = -torque;                                                  // Previous solution
			
			initialGuess.head(6)       = fc;                                            // Constraint forces
			initialGuess.tail(this->n) = qddot;                                         // Use previous solution
			
			torque = M*(solve(H,f,B,z,initialGuess)).tail(this->n);
		}
		
		for(int i = 0; i < this->n; i++) controlInput(i) = torque(i) + coriolisAndGravity(i);  // Add on gravity, convert to iDynTree object
	}
	
	send_torque_commands(controlInput);                                                         // Send the commands to the joint motors
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
 //                 Get the desired accelerations and Cartesian *forces*                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd Humanoid::get_feedforward_feedback(Eigen::VectorXd          &feedback,
                                                  CartesianTrajectory       &trajectory,
                                                  const iDynTree::Transform &actualPose,
                                                  const iDynTree::Twist     &actualVel,
                                                  const double              &time)
{
	// Solve for the desired state
	iDynTree::Transform  desiredPose;
	iDynTree::Twist      desiredVel;
	iDynTree::SpatialAcc desiredAcc;
	
	trajectory.get_state(desiredPose, desiredVel, desiredAcc, time);                            // Get the desired state from trajectory object
	
	// NOTE: This feedback term is FORCE, not acceleration
	feedback = this->D*iDynTree::toEigen(desiredVel - actualVel)                                // Velocity feedback
	         + this->K*iDynTree::toEigen(get_pose_error(desiredPose, actualPose));              // Position, orientation feedback
	
	return iDynTree::toEigen(desiredAcc);                                                       // Return the feedforward term
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                      Compute pose error                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
iDynTree::Vector6 Humanoid::get_pose_error(const iDynTree::Transform &desired,
                                           const iDynTree::Transform &actual)
{
	iDynTree::Position pos = desired.getPosition() - actual.getPosition();                      // Position error
	
	// Convert rotations to quaternions
	iDynTree::Vector4 qd = (desired.getRotation()).asQuaternion();
	iDynTree::Vector4 qa = (actual.getRotation()).asQuaternion();

	// Compute vector component of qd*conj(qa)
	iDynTree::Vector3 rot;
	rot(0) = qa(0)*qd(1) - qd(0)*qa(1) - ( qd(2)*qa(3) - qd(3)*qa(2) );
	rot(1) = qa(0)*qd(2) - qd(0)*qa(2) - ( qd(3)*qa(1) - qd(1)*qa(3) );
	rot(2) = qa(0)*qd(3) - qd(0)*qa(3) - ( qd(1)*qa(2) - qd(2)*qa(1) );
	
	double angle = 2*sqrt(rot(0)*rot(0) + rot(1)*rot(1) + rot(2)*rot(2));                       // Get angle of rotation
	
	
	iDynTree::Vector6 error;
	for(int i = 0; i < 3; i++)
	{
		error(i) = pos(i);
		error(i+3) = 0.0;		
		if(angle < 2*M_PI) error(i+3) =  rot(i);
		else               error(i+3) = -rot(i);                                            // Angle > 180 degrees, spin other way
	}
	
	return error;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //	                  Get the instantaneous joint acceleration limits                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::get_acceleration_limits(double &lower, double &upper, const int &jointNum)
{
	if(jointNum < 0 or jointNum > this->n)
	{
		std::cerr << "[ERROR] [HUMANOID] get_acceleration_limits(): "
		          << "Input " << jointNum << " is outside the joint range." << std::endl;
		
		return false;
	}
	else
	{
		lower = std::max( 2*(this->pMin[jointNum] - this->q[jointNum] - this->dt*qdot[jointNum])/(this->dt*this->dt),
		                   -(this->vMax[jointNum] + this->qdot[jointNum])/this->dt );
		
		upper = std::min( 2*(this->pMax[jointNum] - this->q[jointNum] - this->dt*qdot[jointNum])/(this->dt*this->dt),
		                    (this->vMax[jointNum] - this->qdot[jointNum])/this->dt );
		                        
	        return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //	                 Get the penalty function for joint limit avoidance                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
double Humanoid::get_joint_penalty(const int &jointNum)
{
	double upper = this->pMax[jointNum] - this->q(jointNum);                                    // Distance to upper limit
	double lower = this->q(jointNum)    - this->pMin[jointNum];                                 // Distance from lower limit
	double range = this->pMax[jointNum] - this->pMin[jointNum];                                 // Distance between limits
	
//      double penalty = range*range/(4*upper*lower);                                               // Penalty function, but only need derivative

	double dpdq = (range*range*(2*this->q(jointNum)-this->pMax[jointNum]-this->pMin[jointNum])) // Derivative (i.e. gradient)
	             /(4*upper*upper*lower*lower);
	
	return dpdq;
}

#endif
