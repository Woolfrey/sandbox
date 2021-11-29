#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_
#include <Quintic.h>									// Custom trajectory class
#include <CubicSpline.h>								// Custom trajectory class

class CartesianTrajectory
{
	public:
		CartesianTrajectory() {};						// Default constructor
		
		CartesianTrajectory(	const yarp::sig::Matrix &start_pose,		// Basic constructor
					const yarp::sig::Matrix &end_pose,
					const float &start_time,
					const float &end_time);
					
		void get_state(yarp::sig::Matrix &pose,					// Get the desired state
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &time);
						
	private:		
		Quintic quintic_position;						// Trajectory for the position component
		Quintic quintic_orientation;						// Trajectory for the orientation component
		
		yarp::sig::Matrix R0;							// Starting rotation (needed for later)
		
		// Current state information
		yarp::sig::Vector pos;							// Position (m)
		yarp::sig::Vector axisAngle;						// Axis and angle (rad) of rotation
		yarp::sig::Vector linear_vel;						// Linear velocity (m/s)
		yarp::sig::Vector linear_acc;						// Linear acceleration(m/s/s)
		yarp::sig::Vector angular_vel;						// Angular velocity (rad/s)
		yarp::sig::Vector angular_acc;						// Angular velocity (rad/s)
			
};											// Semicolon needed after class declaration

/******************** Constuctor for 2 poses ********************/
CartesianTrajectory::CartesianTrajectory(const yarp::sig::Matrix &start_pose,		// Basic constructor
					const yarp::sig::Matrix &end_pose,
					const float &start_time,
					const float &end_time)
					:
					R0(start_pose.submatrix(0,2,0,2))		// Initial rotation - needed for later
{
	// Check the inputs are sound
	if(start_pose.rows() 	!= 4
	|| start_pose.cols() 	!= 4
	|| end_pose.rows() 	!= 4
	|| end_pose.cols() 	!= 4)
	{
		yError("CartesianTrajectory::CartesianTrajectory(): Start and end pose must be 4x4 matrices.");
	}
	
	// WHY CAN'T I SET THE SIZE UPON DECLARATION ABOVE?
	this->pos.resize(3);
	this->axisAngle.resize(4);
	this->linear_vel.resize(3);
	this->angular_vel.resize(3);
	this->linear_acc.resize(3);
	this->angular_acc.resize(3);
	
	// Set up the position trajectory (it's simple enough to interpolate from start to end)
	yarp::sig::Vector p1(3), p2(3);
	for(int i = 0; i < 3; i++)
	{
		p1[i] = start_pose[i][3];						// Transfer vectors
		p2[i] = end_pose[i][3];
	}
	this->quintic_position = Quintic(p1, p2, start_time, end_time);			// Create position-based trajectory
	
	// For orientation, interpolate over the **difference** dR = R0^T*Rf
	// such that later we return R(t) = R0*dR(t) => R(tf) = R0*dR*Rf = Rf
	this-> R0 = start_pose.submatrix(0,2,0,2);					// Initial orientation
	yarp::sig::Matrix Rf = end_pose.submatrix(0,2,0,2);				// Final orientation
	yarp::sig::Matrix dR = R0.transposed()*Rf;					// Difference
	
	this->axisAngle = yarp::math::dcm2axis(dR);					// Convert to axis-angle format
	
	this->quintic_orientation = Quintic(axisAngle, start_time, end_time);		// Create trajectory over axis-angle
}

/******************** Get the desired state for the given time ********************/
void CartesianTrajectory::get_state(yarp::sig::Matrix &pose,					// Get the desired state
					yarp::sig::Vector &vel,
					yarp::sig::Vector &acc,
					const float &time)
{
	// Check the inputs are sound
	if(pose.rows() != 4 || pose.cols() != 4)
	{
		yError("CartesianTrajectory::get_state(): Expected a 4x4 matrix for the pose argument.");

	}
	else if(vel.size() != 6 || acc.size() != 6)
	{
		yError("CartesianTrajectory::get_state(): Expected a 6x1 vector for the vel, acc arguments.");
	}
	
	// Get the desired state
	this->quintic_position.get_state(this->pos, this->linear_vel, this->linear_acc, time);
	this->quintic_orientation.get_state(this->axisAngle, this->angular_vel, this->angular_acc, time);
	
	// Orientation trajectory gives the *difference* dR(t),
	//need to compute the actual desired state as R(t) = R0*dR(t)
	yarp::sig::Matrix R = this->R0*yarp::math::axis2dcm(this->axisAngle);
	
	// Transfer translation, velocity, acceleration components
	for(int i = 0; i < 3; i++)
	{
		pose[i][3]	= this->pos[i];						// Set translation part of SE3 matrix
		for(int j = 0; j < 3; j++) pose[i][j] = R[i][j];			// Set rotation part of SE3 matrix
		vel[i] 		= this->linear_vel[i];					// Set linear velocities
		vel[i+3] 	= this->angular_vel[i];					// Set angular velocities
		acc[i]		= this->linear_acc[i];					// Set linear acceleration
		acc[i+3]	= this->angular_acc[i];					// Set angular acceleration
		
		pose[3][i]	= 0.0;							// Ensure that pose is an SE3 matrix
	}
		pose[3][3]	= 1.0;
}
#endif
