    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                                A control class for the iCub robot                              //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef HUMANOID_H_
#define HUMANOID_H_

#include <CartesianTrajectory.h>
#include <Eigen/Dense>                                                                             // Matrix inverse and SVD
#include <iDynTree/Core/EigenHelpers.h>                                                            // Conver iDynTree tensors to Eigen
#include <iDynTree/KinDynComputations.h>                                                           // Class that does inverse dynamics calculations
#include <iDynTree/Model/FreeFloatingState.h>                                                      // iDynTree::FreeFloatingGeneralizedTorques
#include <iDynTree/Model/Model.h>                                                                  // Class that holds basic kinematic & dynamic info
#include <iDynTree/ModelIO/ModelLoader.h>                                                          // Extracts information from URDF
#include <JointInterface.h>                                                                        // Communicates with motors
#include <iDynTree/Core/CubicSpline.h>                                                             // For trajectory stuff
#include <yarp/os/PeriodicThread.h>                                                                // Keeps timing of the control loop

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
		Humanoid(const std::string &fileName);                                             // Constructor

		bool move_to_pose(const iDynTree::Transform &desired,                              // Move one hand to a given pose
				  const std::string &whichHand);
				  
		bool move_to_pose(const iDynTree::Transform &left,                                 // Move both hands to specified poses
				  const iDynTree::Transform &right);
			
		bool move_to_position(const iDynTree::VectorDynSize &position);                    // Move the joints to a given position
		
		bool move_to_positions(const std::vector<iDynTree::VectorDynSize> &positions);     // Move the joints through multiple positions
		
		bool move_to_positions(const std::vector<iDynTree::VectorDynSize> &positions,      // Move the joints to multiple positions, times given
				       const iDynTree::VectorDynSize &times);
		
		bool translate(const iDynTree::Position &distance,                                 // Move a single hand the given distance
			       const std::string &whichHand);
		
		bool translate(const iDynTree::Position &leftDistance,                             // Move both hands
			       const iDynTree::Position &rightDistance);
			       
		bool update_state();                                                               // Update the joint states	
						 
		void halt();                                                                       // Stop the robot immediately
	private:
		double dt = 0.01;                                                                  // Default control frequency
		enum ControlSpace {joint, cartesian, dual} controlSpace;
		iDynTree::VectorDynSize q, qdot;                                                   // Joint position and velocities
		
		// Joint control
		double kq = 50.0;                                                                  // Proportional gain for joint control
		double kd =  2.0;                                                                  // Derivative gain for joint control
		std::vector<iDynTree::CubicSpline> jointTrajectory;                                // Joint trajectory object
		Eigen::VectorXd aMin, aMax;                                                        // Minimum and maximum joint accelerations	
		
		// Cartesian control
		bool leftControl, rightControl;
		CartesianTrajectory leftTrajectory, rightTrajectory;
		Eigen::Matrix<double,6,6> K, D;
		iDynTree::Transform leftHandTF, rightHandTF;
		
		// Kinematics & dynamics
		iDynTree::FreeFloatingGeneralizedTorques generalForces;                            // Forces and torques		
		iDynTree::KinDynComputations computer;                                             // Does all the kinematics & dynamics
		iDynTree::Transform torsoPose;
		iDynTree::Twist torsoTwist;
		iDynTree::Vector3 gravity;
			
		// Control loop stuff
		double startTime;                                                                  // Used to time the control loop
		bool threadInit();                                                                 // From yarp::os::PeriodicThread class
		void run();                                                                        // From yarp::os::PeriodicThread class
		void threadRelease();                                                              // From yarp::os::PeriodicThread class
		
		// Functions
		Eigen::MatrixXd inverse(const Eigen::MatrixXd &A);                                 // Get the inverse of the given matrix
		
		Eigen::VectorXd solve_force_control(const std::string &whichHand,
						    const Eigen::MatrixXd &A,
						    const double &time);

		double get_penalty(const int &j);                                                  // Get the penalty for joint limit avoidance		
};                                                                                                 // Semicolon needed after a class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
Humanoid::Humanoid(const std::string &fileName) :
	           yarp::os::PeriodicThread(0.01),                                                 // Create the threading object for control
                   JointInterface(jointList),                                                      // Open communication with the robot
                   torsoPose(iDynTree::Transform(iDynTree::Rotation::RPY(0,0,0),iDynTree::Position(0,0,0.64))),
                   torsoTwist(iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)))
{	
	// Set the gravity vector
	this->gravity(0) =  0.00;
	this->gravity(1) =  0.00;
	this->gravity(2) = -9.81;
	
	// Set the Cartesian control gains	
	this->K <<  50.0,  0.0,  0.0,  0.0,  0.0,  0.0,
		     0.0, 50.0,  0.0,  0.0,  0.0,  0.0,
		     0.0,  0.0, 50.0,  0.0,  0.0,  0.0,
		     0.0,  0.0,  0.0,  5.0,  0.5,  0.0,
		     0.0,  0.0,  0.0,  0.5,  5.0,  0.0,
		     0.0,  0.0,  0.0,  0.0,  0.0,  5.0;
		     
	this->D <<   5.0,  0.0,  0.0,  0.0,  0.0,  0.0,
		     0.0,  5.0,  0.0,  0.0,  0.0,  0.0,
		     0.0,  0.0,  5.0,  0.0,  0.0,  0.0,
		     0.0,  0.0,  0.0,  1.0,  0.0,  0.0,
		     0.0,  0.0,  0.0,  0.0,  1.0,  0.0,
		     0.0,  0.0,  0.0,  0.0,  0.0,  1.0;

	// Load a model
	iDynTree::ModelLoader loader;                                                              // Temporary

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
					      iDynTree::Transform(iDynTree::Rotation::RPY(0,M_PI/2,0),
							          iDynTree::Position(0,0,0.04)));
		temp.addAdditionalFrameToLink("r_hand", "right",
					      iDynTree::Transform(iDynTree::Rotation::RPY(0,M_PI/2,0),
					      			  iDynTree::Position(0,0,0.04)));
									    
		if(not this->computer.loadRobotModel(temp))
		{
			std::cerr << "[ERROR] [HUMANOID] Constructor: "
				  << "Could not generate iDynTree::KinDynComputations class from given model: "
				  << loader.model().toString() << std::endl;
		}
		else
		{
			this->n = this->computer.model().getNrOfDOFs();                                    // Get number of joints from underlying model
			this->generalForces.resize(this->computer.model());                                // Restructure force class to match model

			// Resize vectors based on model information
			this->aMin.resize(this->n);
			this->aMax.resize(this->n);
			this->jointTrajectory.resize(this->n);
			this->q.resize(this->n);
			this->qdot.resize(this->n);
			this->jointTrajectory.resize(this->n);

			std::cout << "[INFO] [HUMANOID] Successfully created iDynTree model from " << fileName << "." << std::endl;

			update_state();                                                            // Get the current joint state
			
			std::cout << "\nHere is the pose of the left hand:" << std::endl;
			std::cout << (this->computer.getWorldTransform("left").asHomogeneousTransform()).toString() << std::endl;
			
			if(activate_control()) move_to_position(iDynTree::VectorDynSize(startConfiguration));
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
bool Humanoid::move_to_pose(const iDynTree::Transform &desired, const std::string &whichHand)
{
	if(whichHand != "left" and whichHand != "right")
	{
		std::cerr << "[ERROR] [HUMANOID] move_to_pose(): "
			  << "Expected 'left or 'right' as an argument, "
			  << "but your input was " << whichHand << "." << std::endl;
		return false;
	}
	else
	{
		if(isRunning()) stop();                                                            // Stop any control threads that are running
		this->controlSpace = cartesian;                                                    // Switch to Cartesian control mode
		
		iDynTree::VectorDynSize times(2); times(0) = 0.0; times(1) = 3.0;                  // Set up times for the trajectory object
		
		std::vector<iDynTree::Transform> waypoints;
		
		if(whichHand == "left")
		{
			this->leftControl = true;
			this->rightControl = false;
			
			waypoints.push_back( this->leftHandTF );
			waypoints.push_back( desired );
			
			this->leftTrajectory = CartesianTrajectory(waypoints, times);
		}
		else
		{
			this->leftControl = false;
			this->rightControl = true;
			
			waypoints.push_back( this->rightHandTF );
			waypoints.push_back( desired );
			
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
			    const iDynTree::Transform &right)
{
	if(isRunning()) stop();                                                                    // Stop any control threads that are running
	this->controlSpace = cartesian;                                                            // Switch to Cartesian control mode
	
	this->leftControl = true; this->rightControl = true;                                       // Activate control of both hands
	
	iDynTree::VectorDynSize time(2); time[0] = 0.0; time[1] = 3.0;                             // Set up times for the trajectory
	
	std::vector<iDynTree::Transform> waypoint; waypoint.resize(2);
	
	waypoint[0] = this->leftHandTF;                                                            // Current left hand pose
	waypoint[1] = left;                                                                        // Desired left hand pose
	this->leftTrajectory = CartesianTrajectory(waypoint, time);                                // Create new trajectory for left hand
	
	waypoint[0] = this->rightHandTF;                                                           // Current right hand pose
	waypoint[1] = right;                                                                       // Desired right hand pose
	this->rightTrajectory = CartesianTrajectory(waypoint,time);                                // Create new trajectory for right hand
	
	start();
//      threadInit(); this is executed immediately after start() is called
	return true;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //          Move the joints to a desired configuration (optimal time solved automatically)       //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_position(const iDynTree::VectorDynSize &position)
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
		double t = 2.0;                                                                    // Default trajectory time
		for(int i = 0; i < this->n; i++)
		{
			double dq = abs(position[i] - this->q[i]);                                 // Distance to target from current joint position
			double tMin = (3*dq)/(2*this->vMax[i]);                                    // Min. to reach target at max. speed	
			if(tMin > t) t = tMin;                                                     // Override default if its too fast
		}
		
		iDynTree::VectorDynSize time(1); time[0] = t;
		std::vector<iDynTree::VectorDynSize> target; target.push_back(position);           // Add position to a vector object
		return move_to_positions(target, time);                                            // Call the main joint control function
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Move the joints to several desired configurations at given time               //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_positions(const std::vector<iDynTree::VectorDynSize> &positions,
				 const iDynTree::VectorDynSize &times)
{
	if(isRunning()) stop();                                                                    // Stop any control threads that are running
	this->controlSpace = joint;                                                                // Switch to joint control
	
	bool allGood;
	int m = positions.size() + 1;                                                              // 1 extra waypoint for starting position
	iDynTree::VectorDynSize waypoint(m);                                                       // All waypoints of a single joint
	iDynTree::VectorDynSize t(m);                                                              // Times to reach each waypoint
	for(int i = 0; i < this->n; i++)                                                           // ... ith joint
	{
		for(int j = 0; j < m; j++)                                                         // ... jth waypoint
		{
			if(j == 0)
			{
				waypoint[j] = this->q[i];                                          // Use current joint value
				t[j] = 0.0;                                                        // Start immediately
			}
			else
			{
				double target = positions[j-1][i];                                 // jth target of ith joint
				if(target < this->pMin[i])      target = this->pMin[i] + 0.001;    // Just above the lower limit
				else if(target > this->pMax[i]) target = this->pMax[i] - 0.001;    // Just below the upper limit
				waypoint[j] = target;                                              // Assign the target
				t[j] = times[j-1];                                                 // Add on subsequent time data
			}
		}
		allGood = this->jointTrajectory[i].setData(t,waypoint);                            // Set new trajectory data
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
bool Humanoid::translate(const iDynTree::Position &distance, const std::string &whichHand)
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
		iDynTree::Transform T1;
		if(whichHand == "left") T1 = this->leftHandTF;
		else			T1 = this->rightHandTF;

		iDynTree::Transform T2(T1.getRotation(), T1.getPosition() + distance);              // Offset the translation by the given distance

		return move_to_pose(T2, whichHand);                                                 // Move the hand to the given pose
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Translate both hands by the specified amount                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::translate(const iDynTree::Position &leftDistance, const iDynTree::Position &rightDistance)
{
	iDynTree::Transform T1(this->leftHandTF.getRotation(), this->leftHandTF.getPosition() + leftDistance);
	iDynTree::Transform T2(this->rightHandTF.getRotation(), this->rightHandTF.getPosition() + rightDistance);
	
	return move_to_pose(T1, T2);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Update the joint state for all the limbs                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::update_state()
{
	if(JointInterface::read_encoders())
	{
		get_joint_state(this->q, this->qdot);                                              // Get the state as iDynTree objects
		
		if(this->computer.setRobotState(this->torsoPose, this->q, this->torsoTwist, this->qdot, this->gravity))
		{
		
			// Update the pose of each hand
			this->leftHandTF = this->computer.getWorldTransform("l_hand")*leftHandOffset;
			this->rightHandTF = this->computer.getWorldTransform("r_hand")*rightHandOffset;
				
			// Compute new acceleration limits
			for(int i = 0; i < this->n; i++)
			{
				this->aMin[i] = std::max((this->pMin[i] - this->q[i] - this->dt*this->qdot[i])/(this->dt*this->dt),
							 (-this->vMax[i] - this->qdot[i])/this->dt);
							 
				this->aMax[i] = std::min((this->pMax[i] - this->q[i] - this->dt*this->qdot[i])/(this->dt*this->dt),
							 (this->vMax[i] - this->qdot[i])/this->dt);
			}
			
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
	if(isRunning())	stop();                                                                    // Stop any control threads that might be running
	move_to_position(this->q);                                                                 // Run the control to stay at current configuration	
}

 ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                              This is executed just after 'start()' is called                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::threadInit()
{
	this->startTime = yarp::os::Time::now();                                                   // Used to regulate the control loop
	return true;
// 	run(); (this line is automatically executed at the end of this function)
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
			iDynTree::VectorDynSize qddot(this->n),                                    // Reference acceleration
						qddot_d(this->n),                                  // Desired acceleration
						qdot_d(this->n),                                   // Desired velocity
						q_d(this->n);                                      // Desired position
			
			for(int i = 0; i < this->n; i++)
			{
				q_d[i] = this->jointTrajectory[i].evaluatePoint(elapsedTime,       // Get the desired state 
										qdot_d[i],
										qddot_d[i]);
				
				qddot[i] = qddot_d[i]                                              // Feedforward term
					 + this->kd*(qdot_d[i]-this->qdot[i])                      // Velocity feedback
					 + this->kq*(q_d[i]-this->q[i]);                           // Position feedback
			
				if(qddot[i] > this->aMax[i])      qddot[i] = this->aMax[i];        // If over the limit, saturate
				else if(qddot[i] < this->aMin[i]) qddot[i] = this->aMin[i];        // If under the limit, saturate
			}
			
			// Compute the inverse dynamics from the joint accelerations
			iDynTree::Vector6 baseAcc; baseAcc.zero();                                 // Zero base acceleration
			iDynTree::LinkNetExternalWrenches wrench(this->computer.model()); wrench.zero(); // No external wrenches
			this->computer.inverseDynamics(baseAcc, qddot, wrench, this->generalForces); // Solve the inverse dynamics
			tau = this->generalForces.jointTorques();                                  // Extract the joint torque vector
			
			break;
		}
		case cartesian:
		{
			// Generate the Jacobian
			Eigen::MatrixXd J(12,this->n);                                             // Jacobian for both hands
			Eigen::MatrixXd temp(6,6+this->n);                                         // Jacobian for a single hand
			
			this->computer.getFrameFreeFloatingJacobian("l_hand", temp);               // Get the full left hand Jacobian
			J.block(0,0,6,this->n) = temp.block(0,6,6,this->n);                        // Assign to the larger Jacobian
			if(not this->leftControl) J.block(0,0,6,3).setZero();                      // Remove contribution of torso joints
			
			this->computer.getFrameFreeFloatingJacobian("r_hand", temp);               // Get the full right hand Jacobian
			J.block(6,0,6,this->n) = temp.block(0,6,6,this->n);                        // Assign the right hand Jacobian
			if(not this->rightControl) J.block(6,0,6,3).setZero();                     // Remove contribution of torso joints
			
			Eigen::MatrixXd Jt = J.transpose();                                        // Makes things a little easier later
			
			// Compute the joint and Cartesian inertia matrices
			Eigen::MatrixXd M(6+this->n,6+this->n);                                  
			this->computer.getFreeFloatingMassMatrix(M);                               // Inertia including floating base
			M = M.block(6,6,this->n,this->n);                                          // Remove the floating base part
			Eigen::MatrixXd invM = inverse(M);                                         // Invert the inertia matrix
			Eigen::MatrixXd A = inverse(J*invM*Jt);                                    // Cartesian inertia matrix
			
			// Solve the Cartesian force control
			Eigen::VectorXd f(12); f.setZero();
			if(this->leftControl)  f.block(0,0,6,1) = solve_force_control("l_hand", A.block(0,0,6,6), elapsedTime);
			if(this->rightControl) f.block(6,0,6,1) = solve_force_control("r_hand", A.block(6,6,6,6), elapsedTime);

			Eigen::VectorXd tau_R = Jt*f;                                              // Range space task
			
			// Compute the desired torques and null space task
			Eigen::VectorXd tau_d(this->n);                                            // Desired torque
			for(int i = 0; i < this->n; i++)
			{
				tau_d(i) = -get_penalty(i) - this->kd*this->qdot[i];
			}
			Eigen::VectorXd tau_N = (Eigen::MatrixXd::Identity(this->n,this->n) - Jt*A*J*invM)*tau_d; // Null space task
			
			this->computer.generalizedBiasForces(this->generalForces);			
			Eigen::VectorXd coriolisAndGravity = iDynTree::toEigen(this->generalForces.jointTorques());	
				
			for(int i = 0; i < this->n; i++) tau(i) = tau_R(i) + tau_N(i) + coriolisAndGravity(i);
			
			break;
		}
		default:
		{
			this->computer.generalizedGravityForces(this->generalForces);
			tau = this->generalForces.jointTorques();
			for(int i = 0; i < this->n; i++) tau(i) -= this->kq*this->qdot(i);
			break;
		}
	}
	
	send_torque_commands(tau);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                           This is executed just after 'stop()' is called                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::threadRelease()
{
	this->computer.generalizedGravityForces(this->generalForces);                              // Get the torque needed to withstand gravity
//	send_torque_commands(this->generalForces.jointTorques());                                  // Send to the robot
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Solve the Cartesian force control                              //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd Humanoid::solve_force_control(const std::string &whichHand,
					      const Eigen::MatrixXd &A,
					      const double &time)
{
	if(whichHand != "l_hand" and whichHand != "r_hand")
	{
		std::cerr << "[ERROR] [HUMANOID] solve_force_control(): "
			  << "Expected 'l_hand' or 'r_hand' as an argument "
			  << "but your input was " << whichHand << "." << std::endl;
		
		return Eigen::VectorXd::Zero(6);
	}
	else
	{
		// Get the desired state
		iDynTree::Transform desiredPose;
		iDynTree::Twist vel;
		iDynTree::SpatialAcc acc;

		if(whichHand == "l_hand") this->leftTrajectory.get_state(desiredPose, vel, acc, time);
		else			  this->rightTrajectory.get_state(desiredPose, vel, acc, time);
		
		Eigen::VectorXd  xdot_d = iDynTree::toEigen(vel);
		Eigen::VectorXd xddot_d = iDynTree::toEigen(acc);
		
		// Compute current state
		iDynTree::Transform actualPose = this->computer.getWorldTransform(whichHand);
		iDynTree::Position posError = desiredPose.getPosition() - actualPose.getPosition();
		iDynTree::Vector3 rotError = (desiredPose.getRotation()*(actualPose.getRotation()).inverse()).asRPY();
		Eigen::VectorXd e(6);
		for(int i = 0; i < 3; i++)
		{
			e(i)   = posError(i);
			e(i+3) = rotError(i);
		}
		
		Eigen::VectorXd xdot(6); this->computer.getFrameVel(whichHand, xdot);              // Actual velocity of the hand
		Eigen::VectorXd bias(6); this->computer.getFrameBiasAcc(whichHand, bias);          // Jdot*qdot
		
		return A*(xddot_d - bias) + this->D*(xdot_d - xdot) + this->K*e;
	}
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
 //	                 Get the penalty function for joint limit avoidance                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
double Humanoid::get_penalty(const int &j)
{
	double upper = this->pMax[j] - this->q(j);                                                 // Distance to upper limit
	double lower = this->q(j) - this->pMin[j];                                                 // Distance from lower limit
	double range = this->pMax[j] - this->pMin[j];                                              // Distance between limits
	
//	double p = range*range/(4*upper*lower);                                                    // This is the actual function but we don't need it

	double dpdq = range*range*(2*this->q(j) - this->pMax[j] - this->pMin[j])                   // Gradient
	             /(4*upper*upper*lower*lower);
	             
	// Cap the value if its too large
	if(dpdq > 1e02)       dpdq =  1e02;
	else if(dpdq < -1e02) dpdq = -1e02;
	
//	return dpdq;
	
	// Return penalty if moving toward a limit?
	if(dpdq*this->qdot(j) > 0) return dpdq;
	else                       return 0.0;
}

#endif
