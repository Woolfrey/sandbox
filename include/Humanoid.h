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
#include <iDynTree/KinDynComputations.h>                                                            // Class that does inverse dynamics calculations
#include <iDynTree/Model/FreeFloatingState.h>                                                       // iDynTree::FreeFloatingGeneralizedTorques
#include <iDynTree/Model/Model.h>                                                                   // Class that holds basic kinematic & dynamic info
#include <iDynTree/ModelIO/ModelLoader.h>                                                           // Extracts information from URDF
#include <JointInterface.h>                                                                         // Communicates with motors
#include <iDynTree/Core/CubicSpline.h>                                                              // For trajectory stuff
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
		Humanoid(const std::string &fileName,                                               // Constructor
		         const std::string &controlMode);
		
		bool move_to_pose(const iDynTree::Transform &desired,                               // Move one hand to a given pose in a given time
		                  const std::string &whichHand,
		                  const double &time);
		
		bool move_to_pose(const iDynTree::Transform &left,                                  // Move both hands to given poses in a given time
		                  const iDynTree::Transform &right,
		                  const double &time);
		                  
		bool move_to_poses(const std::vector<iDynTree::Transform> &left,                    // Move both hands through multiple poses
		                   const std::vector<iDynTree::Transform> &right,
		                   const std::vector<double> &time);

		bool move_to_position(const iDynTree::VectorDynSize &position,                      // Move the joints to a given position in a given time
		                      const double &time);
		
		bool translate(const iDynTree::Position &distance,                                  // Translate one hand by the given distance
		               const std::string &whichHand,
		               const double &time);
		
		bool translate(const iDynTree::Position &left,                                      // Translate both hands by the given distance
		               const iDynTree::Position &right,
		               const double &time);
		       
		bool update_state();                                                                // Update the kinematics & dynamics of the robot
		
		void halt();                                                                        // Stop the robot immediately

	private:
		double dt = 0.01;
		
		enum controlMode {joint, cartesian} controlMode;
		
		iDynTree::VectorDynSize q, qdot;                                                    // Joint positions and velocities
		
		std::vector<double> aMin, aMax, vMin, vMax;                                         // Instantaneous imits on joint acceleration, velocity
		
		// Joint control
		double kq = 50.0;
		double kd =  2.0;
		std::vector<iDynTree::CubicSpline> jointTrajectory;
		
		// Cartesian control
		bool leftControl, rightControl;                                                     // Switch for activating left and right hand control
		CartesianTrajectory leftTrajectory, rightTrajectory;                                // Cartesian trajectories for each hand
		Eigen::Matrix<double,6,6> K, D;                                                     // Cartesian stiffness and damping
	
		// Internal functions
		iDynTree::Vector6 get_pose_error(const iDynTree::Transform &desired,                        // Get the pose error between 2 transforms
			                         const iDynTree::Transform &actual);
		
		Eigen::MatrixXd get_inverse(const Eigen::MatrixXd &A);
		
		Eigen::MatrixXd get_weighted_inverse(const Eigen::MatrixXd &A,
			                             const Eigen::MatrixXd &W);
		
		void get_inverse_and_nullspace(const Eigen::MatrixXd &A,
			                       const Eigen::MatrixXd &W,
			                       Eigen::MatrixXd &invA,
			                       Eigen::MatrixXd &N);
		
		// Control loop functions related to the PeriodicThread class
		double startTime;
		bool threadInit() {return true;}
		void run() {}
		void threadRelease() {}
};                                                                                                  // Semicolon needed after a class declaration


#endif
