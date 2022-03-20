    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                                  A control class for the iCub robot                            //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef HUMANOID_H_
#define HUMANOID_H_

#include <CartesianTrajectory.h>                                                                   // Custom trajectory class
#include <Cubic.h>                                                                                 // Custom trajectory class
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
		bool update_state();                                                               // Update the joint states
		bool move_to_pose(const iDynTree::Transform &pose, const std::string &whichHand);  // Move a single hand to a desired pose
		bool move_to_pose(const iDynTree::Transform &left, const iDynTree::Transform &right);
		bool move_to_position(const iDynTree::VectorDynSize &position);                    // Move to a desired configuration
		bool move_to_positions(const std::vector<iDynTree::VectorDynSize> &positions);     // Move through several joint configurations
		bool move_to_positions(const std::vector<iDynTree::VectorDynSize> &positions,
		                       const std::vector<double> &times);
		void halt();                                                                       // Stop any control and maintain current position
		
	private:
		bool isValid = true;                                                               // Will not do computations if true
		bool leftControl, rightControl;
		double dt = 0.01;                                                                  // Discrete time step
		enum ControlSpace {joint, cartesian, dual} controlSpace;                           // Switch cases
		
		iDynTree::FreeFloatingGeneralizedTorques generalForces;                            // Forces and torques
		
		// Joint Control
		double Kq = 50;                                                                    // Proportional gain
		double Kd = 5.0;                                                                   // Derivative gain
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
		double get_penalty(const int &j);                                                  // Get the penalty for joint limit avoidance
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
                   torsoPose(iDynTree::Transform(iDynTree::Rotation::RPY(0,0,M_PI),iDynTree::Position(0,0,0.65))),
                   torsoTwist(iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0)))
{
	// Set the gravity vector
	this->gravity(0) = 0.0;
	this->gravity(1) = 0.0;
	this->gravity(2) =-9.81;
	
	// Set the Cartesian control gains
	this->K.resize(6,6);
	this->K(0,0) = 50.0;
	this->K(1,1) = 50.0;
	this->K(2,2) = 50.0;
	this->K(3,3) = 5.0;
	this->K(3,4) = 0.5;
	this->K(4,3) = 0.5;
	this->K(4,4) = 5.0;
	this->K(5,5) = 5.0;

	this->D.resize(6,6);
	for(int i = 0; i < 3; i++)
	{
		this->D(i,i)     = 5.0;
		this->D(i+3,i+3) = 0.5;
	}

	// Load a model
	iDynTree::ModelLoader loader;                                                              // Temporary

	if(not loader.loadReducedModelFromFile(fileName, jointList, "urdf"))
	{
		std::cerr << "[ERROR] [HUMANOID] Constructor: "
		          << "Could not load model from path " << fileName << std::endl;
		this->isValid = false;
	}
	else
	{
		if(not this->computer.loadRobotModel(loader.model()))
		{
			std::cerr << "[ERROR] [HUMANOID] Constructor: "
			          << "Could not generate iDynTree::KinDynComputations class from given model: "
			          << loader.model().toString() << std::endl;
			this->isValid = false;
		}
		else
		{
			this->model = computer.model();                                            // Get the model from the computer
			this->n = model.getNrOfDOFs();                                             // Degrees of freedom / number of joints
			this->generalForces.resize(this->model);                                   // Restructure force class to match model
			
			std::cout << "[INFO] [HUMANOID] Successfully created iDynTree model from " << fileName << "." << std::endl;
	
			update_state();                                                            // Get the current joint state
			if(activate_control())
			{
				move_to_position(iDynTree::VectorDynSize(startConfiguration));
			}
			else
			{
				std::cerr << "[ERROR] [HUMANOID] Constructor: "
			                  << "Could not activate joint control." << std::endl;
			        this->isValid = false;
			}
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
		std::cerr << "[ERROR] [HUMANOID] get_hand_pose():"
		          << "String input was " << whichHand << " but expected 'left' or 'right' as an argument. "
		          << "Returning left hand pose by default." << std::endl;
		          
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
	iDynTree::Position pos = desired.getPosition() - actual.getPosition();                     // Position error
	iDynTree::Rotation rot = desired.getRotation()*actual.getRotation().inverse();             // Rotation error
	iDynTree::Vector3 rpy = rot.asRPY();                                                       // Get the error as roll, pitch, yaw angles
	
	for(int i = 0; i < 3; i++)
	{
		error[i]   = pos[i];
		error[i+3] = rpy[i];
	}
	
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
 //                              Move one hand to a desired pose                                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_pose(const iDynTree::Transform &pose, const std::string &whichHand)
{
	if(whichHand != "left" and whichHand != "right")
	{
		std::cerr << "[ERROR] [HUMANOID] move_to_pose(): "
			  << "Expected 'left' or 'right' but your input was " << whichHand << "." << std::endl;
		
		return false;
	}
	else
	{
		if(isRunning()) stop();                                                            // Stop any control threads that are running
		this->controlSpace = cartesian;                                                    // Switch to Cartesian control if not already
		
		if(whichHand == "left")
		{
			this->leftControl = true;                                                  // Activate left hand control
			this->rightControl = false;                                                // Deactive right hand control
			
			this->leftHandTrajectory = CartesianTrajectory(get_hand_pose("left"), pose, 0, 3.0); // Set up the left hand trajectory
		}
		else
		{
			this->leftControl = false;                                                 // Deactive the left hand control
			this->rightControl = true;                                                 // Active the right hand control
			
			this->rightHandTrajectory = CartesianTrajectory(get_hand_pose("right"), pose, 0, 3.0); // Set up the right hand trajectory
		}
		
		start();                                                                           // Restart the control thread
//		threadInit(); this is executed automatically after start is called

		return true;
	}
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
			double tMin = (3*dq)/(2*this->vLim[i]);                                    // Min. to reach target at max. speed
			
			if(tMin > t) t = tMin;                                                     // Override default if its too fast
		}
		
		std::vector<double> time; time.push_back(t);                                       // Add time to a vector object
		std::vector<iDynTree::VectorDynSize> target; target.push_back(position);           // Add position to a vector object
		return move_to_positions(target, time);                                            // Call the main joint control function
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //           Move the joints to several desired configurations (times solved automatically)      //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_positions(const std::vector<iDynTree::VectorDynSize> &positions)
{
	bool OK = true;
	
	// Check the input dimensions are sound
	for(int i = 0; i < positions.size(); i++)
	{
		if(positions[i].size() != this->n)
		{
			std::cerr << "[ERROR] [HUMANOID] move_to_positions(): "
				  << "Position vector " << i+1 << " had " << positions[i].size()
				  << " elements but this model has " << this->n << " joints." << std::endl;
			OK = false;
		}
	}
	
	if(OK)
	{
		std::vector<double> times;                                                         // We need to solve for this
		
		// Solve an "optimal" time
		for(int i = 0; i < positions.size(); i++)
		{
			double t = 2.0;                                                           // Default time
			for(int j = 0; j < this->n; j++)
			{
				double dq;
				if(i == 0) dq = abs(positions[i][j] - this->q[j]);                 // Distance from current joint positions          
				else       dq = abs(positions[i][j] - positions[i-1][j]);          // Distance from previous waypoint
				
				double tMin = (3*dq)/(2*this->vLim[j]);                            // Minimum time if average speed is 66% of max
				
				if(tMin > t) t = tMin;                                             // Override if default time is too fast
			}
			
			if(i == 0) times.push_back(t);                                             // Assume we start from time t = 0
			else       times.push_back(times[i-1] + t);                                // Increment the previous time
		}
		
		return move_to_positions(positions,times);                                         // Call the full joint control function
	}
	else return false;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                Move the joints to several desired configurations (times given)                //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool Humanoid::move_to_positions(const std::vector<iDynTree::VectorDynSize> &positions, const std::vector<double> &times)
{
	bool OK = true;
	
	// Check that the number of waypoints is equal to the number of times
	if(positions.size() != times.size())
	{
		std::cout << "[ERROR] [HUMANOID] move_to_positions(): "
			  << "Position vector had " << positions.size() << " waypoints, "
			  << "times had " << times.size() << " elements." << std::endl;
		OK = false;
	}
	else
	{
		// Check the time vector is in order
		for(int i = 0; i < times.size()-1; i++)
		{
			if(times[i] == times[i+1])
			{
				std::cerr << "[ERROR] [HUMANOID] move_to_positions(): "
					  << "Time " << i+1 << " and time " << i+2 << " are the same! "
					  << "(" << times[i] << " seconds). Cannot move in zero time." << std::endl;
				OK = false;
			}
			else if(times[i] > times[i+1])
			{
				std::cerr << "[ERROR] [HUMANOID] move_to_positions(): "
					  << "Times are not in ascending order. Time " << i+1 << " was "
					  << times[i] << " seconds and time " << i+2 << " was "
					  << times[i+1] << " seconds." << std::endl;
				OK = false;
			}
		}
		
		// Check that the number of dimensions are correct
		for(int i = 0; i < positions.size(); i++)
		{
			if(positions[i].size() != this->n)
			{
				std::cerr << "[ERROR] [HUMANOID] move_to_positions(): "
					  << "Position vector " << i+1 << " had " << positions[i].size()
					  << " elements but this model has " << this->n << " joints." << std::endl;
				OK = false;
			}
		}
	}
	
	if(OK)
	{
		// Transfer the points and cap them if they are outside joint limits
		std::vector<iDynTree::VectorDynSize> waypoint; waypoint.push_back(this->q);        // Use current joint position as first waypoint
		std::vector<double> t; t.push_back(0.0);                                           // Start time of zero
		
		for(int i = 0; i < positions.size(); i++)
		{
			waypoint.push_back(positions[i]);                                          // Add position to end of vector
			t.push_back(times[i]);
			for(int j = 0; j < this->n; j++)
			{
				if(waypoint[i+1][j] < this->qMin[j])
				{
					waypoint[i+1][j] = this->qMin[j] + 0.001;                 // Just above the lower limit
					
					std::cout << "[WARNING] [HUMANOID] move_to_positions() "
						  << "Target position for joint " << j+1 << " of "
						  << positions[i][j]*180/M_PI << " degrees was below the joint limit of "
						  << this->qMin[j]*180/M_PI << " degrees." << std::endl;
				}
				else if(waypoint[i+1][j] > this->qMax[j])
				{
					waypoint[i+1][j] = this->qMax[j] - 0.001;                  // Just below the lower limit
					
					std::cout << "[WARNING] [HUMANOID] move_to_position() "
						  << "Target position for joint " << j+1 << " of "
						  << positions[i][j]*180/M_PI << " degrees was above the joint limit of "
						  << this->qMax[j]*180/M_PI << " degrees." << std::endl;
				}
			}
		}
		
		if(isRunning()) stop();                                                            // Stop any control threads that are running
		this->controlSpace = joint;                                                        // Switch to joint control mode
		this->jointTrajectory = Cubic(waypoint,t);                                         // New Cubic spline trajectory
		start();                                                                           // Start the control loop
//		threadInit(); (this line is automatically executed when start() is called)

		return true;
	}
	else return false;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Hold the current joint positions                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::halt()
{
	if(isRunning())	stop();                                                                    // Stop any control threads that might be running
	update_state();                                                                            // Get the current joint state
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
			iDynTree::VectorDynSize qddot(this->n);                                    // Reference acceleration to be computed
			iDynTree::VectorDynSize q_d(this->n), qdot_d(this->n), qddot_d(this->n);   // Desired state
			
			if(this->jointTrajectory.get_state(q_d, qdot_d, qddot_d, elapsedTime))
			{
				for(int i = 0; i < this->n; i++)
				{
					// Compute the joint control
					qddot[i] = qddot_d[i]                                      // Feedforward term
						 + this->Kd*(qdot_d[i] - this->qdot[i])            // Derivative gain
						 + this->Kq*(q_d[i] - this->q[i]);                 // Proportional gain
					
					// Compute the joint limits
					double aMax = std::min((this->qMax[i] - this->q[i] - this->dt*this->qdot[i])/(this->dt*this->dt),
							       (this->vLim[i] - this->qdot[i])/this->dt);
					double aMin = std::max((this->qMin[i] - this->q[i] - this->dt*this->qdot[i])/(this->dt*this->dt),
							       (-this->vLim[i] - this->qdot[i])/this->dt);

					// Cap the control to remain in limits
					if(qddot[i] > aMax)      qddot[i] = aMax;                  // Just below the maximum
					else if(qddot[i] < aMin) qddot[i] = aMin;                  // Just above the minimum
				}
			}
			else
			{
				for(int i = 0; i < this->n; i++) qddot[i] = -this->Kd*this->qdot[i]; // Try not to move!
			}

			// Compute the inverse dynamics from the joint accelerations
			iDynTree::Vector6 baseAcc; baseAcc.zero();                                   // Don't move the base
			iDynTree::LinkNetExternalWrenches wrench(this->model); wrench.zero();        // No external forces applied
			this->computer.inverseDynamics(baseAcc, qddot, wrench, this->generalForces); // Solve the inverse dynamics
			tau = this->generalForces.jointTorques();                                    // Get the joint torques
			
			break;
		}
		case cartesian:
		{
			// Generate the Jacobian
			Eigen::MatrixXd J(12,this->n);                                             // Jacobian for both hands
			Eigen::MatrixXd temp(6,6+this->n);                                         // Jacobian for a single hand
			
			this->computer.getFrameFreeFloatingJacobian("l_hand", temp);               // Get the full left hand Jacobian
			J.block(0,0,6,this->n) = temp.block(0,6,6,this->n);                        // Assign the left hand Jacobian
			if(not this->leftControl) J.block(0,0,6,3).setZero();                      // Remove contribution of torso joints
			
			this->computer.getFrameFreeFloatingJacobian("r_hand", temp);               // Get the full right hand Jacobian
			J.block(6,0,6,this->n) = temp.block(0,6,6,this->n);                        // Assign the right hand Jacobian
			if(not this->rightControl) J.block(6,0,6,3).setZero();                     // Remove contribution of torso joints
			
			// Compute the joint and Cartesian inertia matrices
			Eigen::MatrixXd M(6+this->n,6+this->n);                                  
			this->computer.getFreeFloatingMassMatrix(M);                               // Inertia including floating base
			M = M.block(6,6,this->n,this->n);                                          // Remove the floating base part
			Eigen::MatrixXd invM = inverse(M);                                         // Invert the inertia matrix
			Eigen::MatrixXd A = inverse(J*invM*J.transpose());                         // Cartesian inertia matrix
			
			// Solve the Cartesian control
			iDynTree::VectorDynSize f(12); f.zero();                                   // Force vector to be computed
			iDynTree::Transform pose;                                                  // Desired pose for a hand
			iDynTree::Twist vel;                                                       // Desired velocity for a hand
			iDynTree::SpatialAcc acc;                                                  // Desired acceleration for a hand
			iDynTree::Vector6 bias;                                                    // Bias acceleration Jdot*qdot
			iDynTree::Vector6 e; e.zero();                                             // Pose error
			iDynTree::Vector6 edot; edot.zero();                                       // Velocity error
			
			// Left hand is active, so compute feedforward + feedback control
			if(this->leftControl)
			{	
				this->leftHandTrajectory.get_state(pose, vel, acc, elapsedTime);
				e = get_pose_error(pose, get_hand_pose("left"));
				bias = this->computer.getFrameBiasAcc("l_hand");
				
				// Compute the velocity error
				for(int i = 0; i < 6; i++)
				{
					for(int j = 0; j < 10; j++) edot(i) += J(i,j)*this->qdot(j);
				}
				
				// Compute the force vector
				for(int i = 0; i < 6; i++)
				{
					for(int j = 0; j < 6; j++) f(i) += A(i,j)*(acc(j)-bias(j)) + this->K(i,j)*e(j) + this->D(i,j)*edot(j);
				}
				
			}
			
			// Right hand control is active, so compute feedforward + feedback control
			if(this->rightControl)
			{
				this->rightHandTrajectory.get_state(pose, vel, acc, elapsedTime);
				e = get_pose_error(pose, get_hand_pose("right"));
				bias = this->computer.getFrameBiasAcc("l_hand");
				
				// Compute the velocity error
				for(int i = 0; i < 6; i++)
				{
					for(int j = 0; j < 10; j++)
					{
						if(j < 3) edot(i) += J(i+6,j)*this->qdot(j);
						else      edot(i) += J(i+6,j+10)*this->qdot(j+10);
					}
				}
				
				// Compute the force fector
				for(int i = 0; i < 6; i++)
				{
					for(int j = 0; j < 6; j++) f(i+6) += A(i+6,j+6)*(acc(j)-bias(j)) + this->K(i,j)*e(j) + this->D(i,j)*edot(j);
				}
			}

			// Solve the Cartesian impedance control
			this->computer.generalizedGravityForces(this->generalForces);              // Compute gravitational forces
			iDynTree::VectorDynSize g = this->generalForces.jointTorques();            // Extract the gravity torques
			Eigen::MatrixXd N = Eigen::MatrixXd::Identity(this->n, this->n) - J.transpose()*A*J*invM;
			
			iDynTree::VectorDynSize tau_d(this->n);
			for(int i = 0; i < 12; i++) tau_d(i) = get_penalty(i);
			
			for(int i = 0; i < this->n; i++)
			{
				tau(i) += g(i);                                                    // Gravity compensation

				for(int j = 0; j < this->n; j++)
				{
					if(j < 12) tau(i) += J(j,i)*f(j);                          // Range space torques
					tau(i) -= N(i,j)*(tau_d(j) + 2.0*this->qdot(j));           // Null space torques
				}
			}

			break;
		}
		default:
		{
			this->computer.generalizedGravityForces(this->generalForces);
			tau = this->generalForces.jointTorques();
			haiku();
		}
	}

	// Send the commands to the motors
	send_torque_commands(tau);
//	if(stop()) threadRelease();
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                           This is executed just after 'stop()' is called                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
void Humanoid::threadRelease()
{
	this->computer.generalizedGravityForces(this->generalForces);                              // Get the torque needed to withstand gravity
	send_torque_commands(this->generalForces.jointTorques());                                  // Send to the robot
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
	double upper = this->qMax(j) - this->q(j);                                                 // Distance to upper limit
	double lower = this->q(j) - this->qMin(j);                                                 // Distance from lower limit
	double range = this->qMax(j) - this->qMin(j);                                              // Distance between limits
	
//	double p = range*range/(4*upper*lower);                                                    // This is the actual function but we don't need it

	double dpdq = range*range*(2*this->q(j) - this->qMax(j) - this->qMin(j))                   // Gradient
	             /(4*upper*upper*lower*lower);
	             
	// Cap the value if its too large
	if(dpdq > 1e02)       dpdq =  1e02;
	else if(dpdq < -1e02) dpdq = -1e02;
	
//	return dpdq;
	
	// Return penalty if moving toward a limit?
	if(dpdq*this->qdot(j) > 0) return dpdq;
	else                       return 0.0;
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
	this->computer.generalizedBiasForces(this->generalForces);
	iDynTree::VectorDynSize h = this->generalForces.jointTorques();
	std::cout << "\nHere are the Coriolis and gravity torques:" << std::endl;
	std::cout << h.toString() << std::endl;

	// Compute nonlinear acceleration term
	iDynTree::Vector6 Jdotqdot = this->computer.getFrameBiasAcc("l_hand");
	std::cout << "\nHere is the d/dt(J)*dq/dt:" << std::endl;
	std::cout << Jdotqdot.toString() << std::endl;
}

#endif
