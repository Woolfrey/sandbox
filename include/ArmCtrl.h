#include <iostream>
#include <CartesianTrajectory.h>							// Custom trajectory object
#include <iCub/iKin/iKinFwd.h>								// iCub::iKin::iCubArm object
#include <JointCtrl.h>									// Custom low-level interface with robot


class ArmCtrl :	public JointCtrl
{
	public:
		// Constructor(s)
		ArmCtrl() {};
		
		// Generation Functions
		bool configure(const std::string &local_port_name,
				const std::string &remote_port_name,
				const std::string &_name);
		void prep_for_hessian() {return this->arm.prepareForHessian();}		// Need to call this before get_partial_jacobian()
		
		
		// Set Functions
		bool set_cartesian_trajectory(const yarp::sig::Matrix &desired, const double &time);
		bool set_joint_angles(const yarp::sig::Vector &angles);
		
		// Get Functions
		yarp::sig::Matrix get_jacobian() {return this->arm.GeoJacobian();}	// As it says on the label
		yarp::sig::Matrix get_partial_jacobian(const int &j);			// Returns partial derivative of Jacobian w.r.t joint j
		yarp::sig::Matrix get_pose() {return this->T;}				// Get the SE(3) matrix for the pose of the hand
		yarp::sig::Vector get_cartesian_control(const double &time);		// Feedforward + feedback on trajectory object
		yarp::sig::Vector get_pose_error(const yarp::sig::Matrix &desired, const yarp::sig::Matrix &actual); // Get the pose error
	
	private:
		CartesianTrajectory trajectory;						// Obvious
		iCub::iKin::iCubArm arm;						// Arm kinematics
		yarp::sig::Matrix T;							// SE(3) matrix for the hand
		yarp::sig::Matrix J, dJdq;						// Jacobian & partial derivative
		yarp::sig::Matrix pos;							// Desired pose for the hand
		yarp::sig::Vector vel, acc;						// Desired velocity, acceleration for the hand
		
};											// Semicolon needed after class declaration

/******************** Configure the left arm ********************/
bool ArmCtrl::configure(const std::string &local_port_name,
				const std::string &remote_port_name,
				const std::string &_name)
{	
	// Resize vectors and matrix
	this->pos.resize(4,4);
	this->vel.resize(6);
	this->acc.resize(6);
	
	// Set the iCubArm object
	this->arm = iCub::iKin::iCubArm(_name+"_V3");
	this->arm.setAllConstraints(false);						// I don't know what this does
	for(int i = 0; i < 3; i++) this->arm.releaseLink(i);				// Release all the torso joints for us
	
	return configure_drivers(local_port_name, remote_port_name, _name+" arm", 7);	// We only need 7 joints in the arm
}

/******************** Set new Cartesian trajectory to track ********************/
bool ArmCtrl::set_cartesian_trajectory(const yarp::sig::Matrix &desired, const double &time)
{
	// NOTE: Ensure joint state is updated properly BEFORE calling this function!
	
	if(desired.rows() != 4 || desired.rows() != 4)
	{
		yError("ArmCtrl::set_cartesian_trajectory() : Expected a 4x4 homeogenous transform as an input");
		return false;
	}
	else
	{
		this->trajectory = CartesianTrajectory(this->arm.getH(), desired, 0.0, time);
		return true;
	}
}

/******************** Update the iKinChain object to the current state ********************/
bool ArmCtrl::set_joint_angles(const yarp::sig::Vector &angles)
{
	this->arm.setAng(angles);							// Set the values in the iKinChain object
	this->T = this->arm.getH();							// Update the pose of the hand
	return true;
}

/******************** Get the partial derivative of the Jacobian w.r.t the jth joint ********************/
yarp::sig::Matrix ArmCtrl::get_partial_jacobian(const int &j)
{
	yarp::sig::Matrix dJdq(6,10);							// To be returned
	for(int i = 0; i < this->n; i++) dJdq.setCol(i, this->arm.fastHessian_ij(i,j));
	return dJdq;
}

/******************** Solves the task velocities in the hand frame ********************/
yarp::sig::Vector ArmCtrl::get_cartesian_control(const double &time)
{
	this->trajectory.get_state(this->pos, this->vel, this->acc, time);		// Get the desired state for the current time	
	return vel + 5.0*(get_pose_error(pos, this->T));				// Feedforward + feedback
}

/******************** Get the pose error to perform feedback control ********************/
yarp::sig::Vector ArmCtrl::get_pose_error(const yarp::sig::Matrix &desired, const yarp::sig::Matrix &actual)
{
	yarp::sig::Matrix Re = desired.submatrix(0,2,0,2)*actual.submatrix(0,2,0,2).transposed(); // Rotation error as SO(3)
	yarp::sig::Vector axisAngle = yarp::math::dcm2axis(Re);				// Convert SO(3) to axis-angle
	yarp::sig::Vector error(6);							// Vector to be returned
	for(int i = 0; i < 3; i++)
	{
		error[i]	= desired[i][3] - actual[i][3];				// Translation error
		error[i+3]	= axisAngle[3]*axisAngle[i];				// Orientation error as angle*axis
	}
	return error;
}
