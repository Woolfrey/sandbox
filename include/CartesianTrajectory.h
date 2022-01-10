#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_

#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <Quintic.h>									// Custom trajectory class


class CartesianTrajectory
{
	public:
		// Constructors
		CartesianTrajectory() {}						// Empty constructor
		
		CartesianTrajectory(const iDynTree::Transform &startPose,		// Proper constructor
				const iDynTree::Transform &endPose,
				const double &startTime,
				const double &endTime);
	
		bool get_state(iDynTree::Transform &pose,
				iDynTree::Twist &vel,
				iDynTree::SpatialAcc &acc,
				const double &time);
	private:
		iDynTree::Transform T0;							// Initial transform
		
		Quintic quinticPosition, quinticOrientation;				// Trajectories between 2 points
		
};											// Semicolon needed after class declaration

/******************** Constuctor for 2 given poses ********************/
CartesianTrajectory::CartesianTrajectory(const iDynTree::Transform &startPose,
					const iDynTree::Transform &endPose,
					const double &startTime,
					const double &endTime)
					: T0(startPose)
{
	// Check the inputs are sound
	if(startTime > endTime)
	{
		std::cerr	<< "[ERROR][CARTESIANTRAJECTORY] CartesianTrajectory() : "
				<< " Start time " << startTime << " is greater than end time!" << std::endl;
	}
	
	// Create the position trajectory
	iDynTree::VectorDynSize p1(3); p1 = startPose.getPosition();			// Need to do it this way because iDynTree is annoying
	iDynTree::VectorDynSize p2(3); p2 = endPose.getPosition();
	this->quinticPosition = Quintic(p1, p2, startTime, endTime);
	
	// Create the orientation trajectory
	// dR = R1'*R2;									// Get the difference in rotation
	// angle, axis <- dR								// Extract the angle, axis representation
	// this->quinticOrientation = Quintic(0, angle*axis, startTime, endTime);	// Interpolate over the angle*axis
}

/******************** Get the desired state for the given time ********************/
bool CartesianTrajectory::get_state(iDynTree::Transform &pose, iDynTree::Twist &vel, iDynTree::SpatialAcc &acc, const double &time)
{
	// Variables used in this scope
	iDynTree::VectorDynSize pos(3), linearVel(3), linearAcc(3);
	iDynTree::VectorDynSize rot(3), angularVel(3), angularAcc(3);
		
	if(!this->quinticPosition.get_state(pos, linearVel, linearAcc, time)
	|| !this->quinticOrientation.get_state(rot, angularVel, angularAcc, time))
	{
		std::cerr	<< "[ERROR][CARTESIANTRAJECTORY] get_state() : "
				<< "Could not obtain the desired state for the position or orientation." << std::endl;
				
		pose = this->T0;							// Remain at the start
		vel = iDynTree::Twist(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0));
		acc = iDynTree::SpatialAcc(iDynTree::GeomVector3(0,0,0), iDynTree::GeomVector3(0,0,0));
		
		return false;
	}
	else
	{
		// Need to split rot (which is angle*axis) in to its separate components
		double angle = 0;
		for(int i = 0; i < 3; i++) angle += rot[i]*rot[i];			// Sum of squares
		angle = sqrt(angle);							// Complete the norm
		iDynTree::Direction axis(rot[0], rot[1], rot[2]);			// Get the axis
		axis.Normalize();							// Normalize for good measure
		iDynTree::Rotation R = this->T0.getRotation()*iDynTree::Rotation::RotAxis(axis, angle); // R(t) = R(0)*R(t)
		
		// Combine the linear and angular components
		pose = iDynTree::Transform(R, iDynTree::Position(pos[0], pos[1], pos[2]));
		
		vel = iDynTree::Twist(iDynTree::GeomVector3(linearVel[0], linearVel[1], linearVel[2]),
				      iDynTree::GeomVector3(angularVel[0], angularVel[1], angularVel[2]));
				      
		acc = iDynTree::SpatialAcc(iDynTree::GeomVector3(linearAcc[0], linearAcc[1], linearAcc[2]),
					   iDynTree::GeomVector3(angularAcc[0], angularAcc[1], angularAcc[2]));
		return true;
	}
}

#endif
/*
// OLD VERSION USING yarp::sig 

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
		yarp::sig::Matrix rot;							// Rotation matrix
		yarp::sig::Vector linearVel;						// Linear velocity (m/s)
		yarp::sig::Vector linearAcc;						// Linear acceleration (m/s/s)
		yarp::sig::Vector angularVel;						// Angular velocity (rad/s)
		yarp::sig::Vector angularAcc;						// Angular acceleration (rad/s/s)
			
};											// Semicolon needed after class declaration

/******************** Constuctor for 2 poses ********************
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
	this->quinticPosition = Quintic(startPoint, endPoint, startTime, endTime);
	
	// Create the orientation trajectory
	this->quinticOrientation = Quintic(startPose.submatrix(0,2,0,2),
					endPose.submatrix(0,2,0,2),
					startTime, endTime);
}

/******************** Get the desired state for the given time ********************
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
	for(int i = 0; i < 3; i++)
	{
		pose[i][3] 	= this->pos[i];						// Assign the translation part
		vel[i]		= this->linearVel[i];					
		vel[i+3]	= this->angularVel[i];
		acc[i]		= this->linearAcc[i];
		acc[i+3]	= this->angularAcc[i];
		pose[3][i]	= 0.0;							// Ensure pose is an SE3 matrix
		
		for(int j = 0; j < 3; j++)
		{
			pose[i][j] = this->rot[i][j];					// Assign the rotation matrix
		}
	}
		pose[3][3]	= 1.0;
}
#endif
*/
