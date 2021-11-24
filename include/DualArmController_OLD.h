#include <CartesianTrajectory.h>
#include <ArmController.h>									// Low-level interface with hardware
#include <yarp/os/PeriodicThread.h>
#include <vector>										// std::vector object

class DualArmController : virtual public yarp::os::PeriodicThread
{
	public:
		DualArmController();
		// Constructor
		/*DualArmController() :	yarp::os::PeriodicThread(0.01),
					J(12,17),
					j(6,10),
					pos(4,4),
					vel(6),
					acc(6),
					err(6),
					xdot(12),
					qdot(17) {};*/
		
		void close();									// Close all device drivers
		
		void move_to_position(	yarp::sig::Vector &left,				// Move the joints to a particular position
					yarp::sig::Vector &right);
					
		void move_to_pose(const yarp::sig::Matrix &left,				// Move both hands simultaneously
				   const yarp::sig::Matrix &right,
				   const double &time);
		
		yarp::sig::Vector get_positions(const std::string &which);
		
		yarp::sig::Matrix get_pseudoinverse(const yarp::sig::Matrix &J,		// Get the weighted pseudoinverse Jacobian
						     const yarp::sig::Matrix &W);
		
		/**************************************************************************/
		virtual bool threadInit();
		virtual void run();
		virtual void threadRelease();
		/**************************************************************************/
		
	private:
		double startTime, elapsedTime;						// Used in control timing
		
		MultiJointController torso;							// Torso joints
		
		std::vector<ArmController> arm;						// Left and right arms
		
		// Variables used in run(), declared here in advance:
		yarp::sig::Matrix J;								// Jacobian for both arms
		yarp::sig::Matrix j;								// Jacobian of a single arm
		yarp::sig::Matrix pos;								// Desired pose
		yarp::sig::Vector vel;								// Desired velocity
		yarp::sig::Vector acc;								// Desired acceleration
		yarp::sig::Vector err;								// Pose error
		yarp::sig::Vector xdot;							// Combined task vector
		yarp::sig::Vector qdot;							// Joint velocities			

};												// Semicolon needed after class declaration


/******************** Move the joints to a given position ********************/
void DualArmController::move_to_position(yarp::sig::Vector &left, yarp::sig::Vector &right)
{
	//this->arm[0].move_to_position(left);
	//this->arm[1].move_to_position(right);
	
	//this->leftArm.move_to_position(left);
	//this->rightArm.move_to_position(right);
}

/******************** Move both hands simultaneously ********************/
void DualArmController::move_to_pose(const yarp::sig::Matrix &left, const yarp::sig::Matrix &right, const double &time)
{

/*
	if(isRunning())	stop();
	
*/

/*
	if(isRunning())	stop();							// Stop any Cartesian-level control threads
	if(torso::isRunning())	torso::stop();							// Stop any control threads on the torso joints
	// if(leftArm::isRunning()) leftArm::stop();
	// if(rightArm::isRunning()) rightArm::stop();

	// Check the dimensions of the inputs
	if(left.rows() == 0)	leftControl = false;						// No input specified, so don't move left arm!
	else			leftControl = true;
	
	if(right.rows() == 0)	rightControl = false;						// No input specified, so don't move right arm!
	else			rightControl = true;
	
	if(leftControl || rightControl)
	{
		if(leftControl && left.rows() == 4 && left.cols() == 4)			// Check input dimensions are sound
		{
			// Generate new trajectory here.
		}
		else yError("DualArmController::move_to_pose() : Expected a 4x4 matrix for the left arm pose.");
		
		if(rightControl && right.rows() == 4 && right.cols() == 4)			// Check input dimensions are sound
		{
			// Generate new trajectory here.
		}
		else yError("DualArmController::move_to_pose() : Expected a 4x4 matrix for the right arm pose.");
		
		start(); // Go immediately to threadInit()
	}
	else
	{
		yError("DualArmController::move_to_pose() : Expected at least one SE3 matrix as an input.");
	}
*/
}

/******************** Executed immediately after start() and just before run() ********************/
bool DualArmController::threadInit()
{
	this->startTime = yarp::os::Time::now();						// Start the timer for the control loop
	return true;
}

/******************** MAIN CARTESIAN CONTROL LOOP ********************/
void DualArmController::run()
{
/*

	yarp::sig::Matrix J(12,17), j(6,10), pos(4,4);
	yarp::sig::Vector error(6), vel(6), acc(6), xdot(12);
	
	J.zero();
	xdot.zero();
	
	this->elapsedTime = yarp::os::Time::now() - this->startTime;				// Time since start() was called
	
	for(int i = 0; i < 2; i++)
	{
		if(arm.control)
		{
			// Solve the Cartesian control
			arm[i].trajectory.get_state(pos, vel, acc, elapsedTime);		// Get the desired Cartesian state
			arm[i].update_state();							// Update the joint state
			arm[i].setAng(arm[i].q*M_PI/180);					// Set the joint angles in radians
			error = arm[i].get_pose_error(pos);					// Get the pose error for the current state
			xdot.subVector(0 + i*6, 5 + i*6) = vel + 2.0*error;			// Feedforward + feedback control
			
			// Construct the Jacobian
			j = arm[i].GeoJacobian();						// Get the Jacobian at the current state
			J.setSubmatrix(j.submatrix(0,5,0,6),0+i*6,0+i*7);			// This portion is for the arm joints
			J.setSubmatrix(j.submatrix(0,5,7,9),0+i*6,14);			// This portion is for the torso joints
		}			
	}
	
	// Solve differential inverse kinematics
	yarp::sig::Matrix W = get_joint_weighting();						// NOTE: This is already the inverse!
	yarp::sig::Matrix invJ = get_pseudoinverse(J,W);					// Get the weighted pseudoinverse Jacobian
	yarp::sig::Matrix I(17,17);
	I.zero();
	yarp::sig::Vector qdot = invJ*xdot + (I - invJ*J)*rdot;
	
	for(int i = 0; i < 2; i++)	arm[i].move_at_speed(qdot.subVector(0+i*7, 6+i*7)*180/M_PI);
					torso.move_at_speed(qdot.subVector(14,16)*180/M_PI);
*/
}

/******************** Executed after stop() is called ********************/
void DualArmController::threadRelease()
{
	// Ensure that all the joints stop moving
	arm[0].move_at_speed(yarp::sig::Vector({0,0,0,0,0,0,0}));
	arm[1].move_at_speed(yarp::sig::Vector({0,0,0,0,0,0,0}));
	torso.move_at_speed(yarp::sig::Vector({0,0,0}));
}


/******************** Get the weighted pseudoinverse Jacobian ********************/
yarp::sig::Matrix DualArmController::get_pseudoinverse(const yarp::sig::Matrix &J, const yarp::sig::Matrix &W)
{
/*	
	if(J.cols() != W.rows() || W.cols() != W.rows())
	{
		yError("DualArmController::get_pseudoinverse() : Incorrect input dimensions. Expected an J (mxn) and W (nxn)");
		yarp::sig::Matrix temp(J.cols(), J.rows());
		temp.zero();
		return temp;
	}
	else
	{
		yarp::sig::Matrix invWJt = W*J.transposed();					// Assumes W is already inverted for now
		yarp::sig::Matrix A = J*invWJt;						// We want to invert this matrix
		
		yarp::sig::Matrix U(A.rows(), A.rows()), V(A.cols(), A.cols());		// Orthogonal matrices
		yarp::sig::Vector s(A.rows());						// List of singular values
		
		yarp::math::SVD(A,U,s,V);							// Get the singular value decomposition

		yarp::sig::Matrix invA(A.rows(), A.cols());
		invA.zero();
		for(int i = 0; i < A.rows(); i++)
		{
			for(int j = 0; j < A.cols(); j++)
			{
				if(s(j) == 0)	//	invA(i,J) += 0				// Ignore singular directions
				else if(s(j) < 1e-03)	invA(i,j) += V(i,j)*1e03;		// Damp near-singular directions
				else			invA(i,j) += V(i,j)/s(j);		// Proper inversion of the first half
			}
		}
		
		return invWJt*invA*U.transposed();						// Return the complete inversion
	}
*/
}

/******************** Get the joint positions ********************/
yarp::sig::Vector DualArmController::get_positions(const std::string &which)
{
/*
	if (which.compare("left")) 	return this->leftArm.get_positions();
	else				return this->rightArm.get_positions();
*/
}

/******************** Close the device drivers ********************/
void DualArmController::close()
{
//	this->leftArm.close();
//	this->rightArm.close();
	this->torso.close();
}

/******************** Empty constructor ********************/
DualArmController::DualArmController() :	yarp::os::PeriodicThread(0.01),
						J(12,17),
						j(6,10),
						pos(4,4),
						vel(6),
						acc(6),
						err(6),
						xdot(12),
						qdot(17),
						arm(2)
{
	// Configure the left arm
	std::string local = "/local/left";
	std::string remote = "/icubSim/left_arm";
	std::string name = "left";
	this->arm[0].configure(local, remote, name);
	
	// Configure the right arm
	//local = "/local/right";
	//remote = "/icubSim/right_arm";
	//name = "right";
	//this->arm[1].configure(local, remote, name); */
	
	// Configure the torso
	local = "/local/torso";
	remote = "/icubSim/torso";
	name = "torso";
	this->torso.configure_drivers(local, remote, name, 3);
}

/******************** Move the joints back to their initial configuration ********************
void DualArmController::move_to_start_position()
{
	yarp::sig::Vector left_target({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	yarp::sig::Vector right_target({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
	yarp::sig::Vector torso_target({0.0, 0.0, 0.0});
	
	this->leftArm->move_to_position(left_target);
	this->rightArm->move_to_position(right_target);
	this->torso->move_to_position(torso_target);
}

/******************** Move one of the hands in different directions ********************
void DualArmController::move_left()
{

}
void DualArmController::move_right()
{

}

*/
