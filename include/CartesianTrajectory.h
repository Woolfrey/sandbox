/*
*	A trajectory across two or more poses (position & rotation) in 3D space.
*/

#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_

#include <iDynTree/Core/GeomVector3.h>							// iDynTree::GeomVector3
#include <iDynTree/Core/Position.h>							// iDynTree::Position
#include <iDynTree/Core/Rotation.h>							// iDynTree::Rotation
#include <iDynTree/Core/SpatialAcc.h>							// iDynTree::SpatialAcc
#include <iDynTree/Core/Transform.h>							// iDynTree::Transform
#include <iDynTree/Core/Twist.h>							// iDynTree::Twist
#include <iDynTree/Core/VectorDynSize.h>						// iDynTree::VectorDynSize
#include <QuinticRotation.h>									// Also includes Quintic.h

class CartesianTrajectory
{
	public:
		CartesianTrajectory () {}
		
		CartesianTrajectory(const iDynTree::Transform &startPose,
				const iDynTree::Transform &endPose,
				const double &startTime,
				const double &endTime);
	
		bool get_state(iDynTree::Transform &pose,
				iDynTree::Twist &vel,
				iDynTree::SpatialAcc &acc,
				const double &time);
	private:
		iDynTree::Transform T0;							// Initial transform
		Quintic transTraj;							// Translation trajectory
		QuinticRotation rotTraj;						// Rotation trajectory
	
};											// Semicolon needed after class declaration

/******************** Constructor for 2 waypoints **********/
CartesianTrajectory::CartesianTrajectory(const iDynTree::Transform &startPose,
				const iDynTree::Transform &endPose,
				const double &startTime,
				const double &endTime)
				: T0(startPose)
{
	// Extract the positions and put them in to vectors
	iDynTree::Position startPoint = startPose.getPosition();
	iDynTree::Position endPoint = endPose.getPosition();
	iDynTree::VectorDynSize p1(3), p2(3);
	for(int i = 0; i < 3; i++)
	{
		p1[i] = startPoint[i];
		p2[i] = endPoint[i];
	}
	
	// Assign the translation and rotation trajectories
	this->transTraj = Quintic(p1, p2, startTime, endTime);
	this->rotTraj = QuinticRotation(startPose.getRotation(), endPose.getRotation(), startTime, endTime);
}

/******************** Get the desired state for the given time **********/
bool CartesianTrajectory::get_state(iDynTree::Transform &pose,
				iDynTree::Twist &vel,
				iDynTree::SpatialAcc &acc,
				const double &time)
{
	// Variables used in this scope
	iDynTree::GeomVector3 linearVel, angularVel, linearAcc, angularAcc;
	iDynTree::Position pos;
	iDynTree::Rotation rot;
	iDynTree::VectorDynSize p(3), v(3), a(3);
	
	// Get the desired state from the trajectory objects
	if(this->transTraj.get_state(p, v, a, time) && this->rotTraj.get_state(rot, angularVel, angularAcc, time))
	{
		for(int i = 0; i < 3; i++)
		{
			pos[i] = p[i];
			linearVel[i] = v[i];
			linearAcc[i] = a[i];
		}
	
		// Put them in to the return values
		pose.setPosition(pos);
		pose.setRotation(rot);
		vel = iDynTree::Twist(linearVel, angularVel);
		acc = iDynTree::SpatialAcc(linearAcc, angularAcc);
		return true;
	}
	else
	{
		std::cerr << "[ERROR] [CARTESIANTRAJECTORY] get_state() : Could not the desired state." << std::endl;
		
		pose = this->T0;								// Remain at the start
		linearVel.zero(); angularVel.zero(); linearAcc.zero(); angularAcc.zero();	// Don't move
		vel = iDynTree::Twist(linearVel, angularVel);
		acc = iDynTree::SpatialAcc(linearAcc, angularAcc);
		return false;	
	}
}

#endif
