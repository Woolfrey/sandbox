#include <Quintic.h>										// Custom trajectory class
#include <CubicSpline.h>									// Custom trajectory class

class CartesianTrajectory
{
	public:
		CartesianTrajectory(	const yarp::sig::Matrix &start_pose,			// Basic constructor
					const yarp::sig::Matrix &end_pose,
					const float &start_time,
					const float &end_time);
					
		void get_state(yarp::sig::Matrix &pose,					// Get the desired state
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &time);
						
	private:		
		Quintic quintic_position;							// Trajectory for the position component
		Quintic quintic_orientation;							// Trajectory for the orientation component
		
		yarp::sig::Matrix R0;								// Starting rotation (needed for later)
		
		// Current state information
		yarp::sig::Vector pos;								// Position (m)
		yarp::sig::Vector axisAngle;							// Axis and angle (rad) of rotation
		yarp::sig::Vector linear_vel;							// Linear velocity (m/s)
		yarp::sig::Vector linear_acc;							// Linear acceleration(m/s/s)
		yarp::sig::Vector angular_vel;						// Angular velocity (rad/s)
		yarp::sig::Vector angular_acc;						// Angular velocity (rad/s)
			
};												// Semicolon needed after class declaration

/******************** Constuctor for 2 poses ********************/
CartesianTrajectory::CartesianTrajectory(const yarp::sig::Matrix &start_pose,		// Basic constructor
					const yarp::sig::Matrix &end_pose,
					const float &start_time,
					const float &end_time)
					:
					R0(start_pose.submatrix(0,2,0,2))			// Initial rotation - needed for later
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
		p1[i] = start_pose[i][3];							// Transfer vectors
		p2[i] = end_pose[i][3];
	}
	this->quintic_position = Quintic(p1, p2, start_time, end_time);			// Create position-based trajectory
	
	// For orientation, interpolate over the **difference** dR = R0^T*Rf
	// such that later we return R(t) = R0*dR(t) => R(tf) = R0*dR*Rf = Rf
	yarp::sig::Matrix dR(3,3);
	dR.zero();					
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			for(int k = 0; k < 3; k++) dR[i][j] += start_pose[k][i]*end_pose[k][j]; // This should be R0^T*Rf
		}
	}
	yarp::sig::Vector axisAngle = yarp::math::dcm2axis(dR);				// Convert SO(3) to axis-angle vector
	this->quintic_orientation = Quintic(axisAngle, start_time, end_time);		// Create orientation-based trajectory

	
	// dp = pf - p0;
	// p(t) = p0 + dp(t) -> p(tf) = p0 + dp(tf) = p0 + pf - p0 = pf
	
	// dR = R0^-1*Rf
	// R(t) = R0*dR(t) -> R(tf) = R0*dR(tf) = R0*R0^-1*Rf = Rf
	
	
	/* The code below works just fine:
	
	// Instead of interpolating from start_pose -> end_pose, inteprolate across the *difference*
	// (this makes it easier for the orientation)
	
	yarp::sig::Matrix dT = yarp::math::SE3inv(start_pose)*end_pose;			// Tf = T0*dT --> dT = T0.inverse()*Tf
	//std::cout << dT[0][3] << " " << dT[1][3] << " " << dT[2][3] << std::endl;
	//std::cout << end_pose[0][3] - start_pose[0][3] << " " << end_pose[1][3] - start_pose[1][3]<< " " << end_pose[2][3]- start_pose[2][3]<< std::endl;
	
	yarp::sig::Vector p1(3), p2(3);
	p1.zero();										// Start from zero
	p2[0] = dT[0][3];									// This should be end_position - start_position
	p2[1] = dT[1][3];
	p2[2] = dT[2][3];
	yarp::sig::Vector temp = yarp::math::dcm2axis(dT);					// Get the axis-angle encoded in the transform
	
	// Create trajectories that interpolate over the difference (starting from zero)
	this->quintic_position 	= Quintic(p1, p2, start_time, end_time);
	this->quintic_orientation	= Quintic(temp, start_time, end_time);
	*/
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
	
	// Transfer translation, velocity, acceleration components
	pose.zero();
	for(int i = 0; i < 3; i++)
	{
		pose[i][3]	= this->pos[i];
		vel[i] 	= this->linear_vel[i];
		vel[i+3] 	= this->angular_vel[i];
		acc[i]		= this->linear_acc[i];
		acc[i+3]	= this->angular_acc[i];
	}
	
	// Orientation trajectory gives the *difference* dR(t),
	//need to compute the actual desired state as R(t) = R0*dR(t)
	yarp::sig::Matrix dR = yarp::math::axis2dcm(this->axisAngle);			// Convert to matrix form
	for(int i = 0; i < 3; i++)
	{
		for(int j = 0; j < 3; j++)
		{
			for(int k = 0; k < 3; k++) pose[i][j] = this->R0[i][k]*dR[k][j];	// This should be R0*dR
		}
	}
}
