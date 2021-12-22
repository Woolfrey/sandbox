#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_
#include <Quintic.h>									// Custom trajectory class
#include <CubicSpline.h>								// Custom trajectory class

class CartesianTrajectory
{
	public:
		// Constructors
		CartesianTrajectory() {};						// Default constructor
		
		CartesianTrajectory(const yarp::sig::Matrix &startPose,			// Basic constructor
				const yarp::sig::Matrix &endPose,
				const float &startTime,
				const float &endTime);
				
		// Get functions	
		void get_state(yarp::sig::Matrix &pose,					// Get the desired state
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &time);
						
	private:		
		Quintic quinticPosition;
		Quintic quinticOrientation;
		
		// State information
		yarp::sig::Vector pos;							// Position / translation (m)
		yarp::sig::Matrix rot;							// Rotation (SO3)
		yarp::sig::Vector linearVel;						// Linear velocity (m/s)
		yarp::sig::Vector linearAcc;						// Linear acceleration (m/s/s)
		yarp::sig::Vector angularVel;						// Angular velocity (rad/s)
		yarp::sig::Vector angularAcc;						// Angular acceleration (rad/s/s
			
};											// Semicolon needed after class declaration

/******************** Constuctor for 2 poses ********************/
CartesianTrajectory::CartesianTrajectory(const yarp::sig::Matrix &startPose,		// Basic constructor
					const yarp::sig::Matrix &endPose,
					const float &startTime,
					const float &endTime)
{
	// Check the inputs are sound
	if(startPose.rows() != 4 || startPose.cols() != 4)
	{
		yError() << "CartesianTrajectory::CartesianTrajectory() : Expected an SE3 matrix for the start pose."
			<< "Your input was" << startPose.rows() << "x" << startPose.cols() << ".";
	}
	if(endPose.rows() != 4 || endPose.cols() != 4)
	{
		yError() << "CartesianTrajectory::CartesianTrajectory() : Expected an SE3 matrix for the start pose."
			<< "Your input was" << endPose.rows() << "x" << endPose.cols() << ".";
	}

	// Resize vectors and matrices
	this->pos.resize(3);
	this->rot.resize(3,3);
	this->linearVel.resize(3);
	this->linearAcc.resize(3);
	this->angularVel.resize(3);
	this->angularAcc.resize(3);
	
	// Create the position trajectory
	yarp::sig::Vector startPoint({startPose[0][3], startPose[1][3], startPose[2][3]});
	yarp::sig::Vector endPoint({endPose[0][3], endPose[1][3], endPose[2][3]});
	this->quinticPosition = Quintic(startPoint,
					endPoint,
					startTime,
					endTime);
	
	// Create the orientation trajectory
	this->quinticOrientation = Quintic(startPose.submatrix(0,2,0,2),
					endPose.submatrix(0,2,0,2),
					startTime,
					endTime);
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
	this->quinticPosition.get_state(this->pos, this->linearVel, this->linearAcc, time);
	this->quinticOrientation.get_state(this->rot, this->angularVel, this->angularAcc, time);
	
	// Combine the information
	pose.setSubmatrix(this->rot, 0,0);						// Assign the rotation part
	for(int i = 0; i < 3; i++)
	{
		pose[i][3] 	= this->pos[i];						// Assign the translation part
		vel[i]		= this->linearVel[i];
		vel[i+3]	= this->angularVel[i];
		acc[i]		= this->linearAcc[i];
		acc[i+3]	= this->angularAcc[i];
		pose[3][i]	= 0.0;							// Ensure pose is an SE3 matrix
	}
		pose[3][3]	= 1.0;
}
#endif
