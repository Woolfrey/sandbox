    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //          A trajectory across two or more poses (position & orientation) in 3D space.           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_

#include <iDynTree/Core/CubicSpline.h>                                                              // Fundamental trajectory object
#include <iDynTree/Core/SpatialAcc.h>                                                               // Linear and angular acceleration
#include <iDynTree/Core/Transform.h>                                                                // Position and orientation
#include <iDynTree/Core/Twist.h>                                                                    // Linear and angular velocity
#include <vector>                                                                                   // std::vector

class CartesianTrajectory
{
	public:
		CartesianTrajectory () {}
		
		CartesianTrajectory(const std::vector<iDynTree::Transform> &waypoint,
				    const iDynTree::VectorDynSize &time);
				    
		bool get_state(iDynTree::Transform &pose,
			       iDynTree::Twist &vel,
			       iDynTree::SpatialAcc &acc,
			       const double &time);
	private:
		bool isValid = false;                                                               // Won't do computations if this is false
		int n;                                                                              // Number of waypoints
		std::vector<iDynTree::CubicSpline> spline;                                          // Array of splines
		bool times_are_sound(const iDynTree::VectorDynSize &time);
	
};                                                                                                  // Semicolon needed after class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianTrajectory::CartesianTrajectory(const std::vector<iDynTree::Transform> &waypoint,
					 const iDynTree::VectorDynSize &time):
					 n(waypoint.size())
{
	// Check the dimensions are sound
	if(waypoint.size() != time.size())
	{
		std::cerr << "[ERROR] [CARTESIANTRAJECTORY] Constructor: "
			  << "Arguments are not of equal length! "
			  << "There were " << waypoint.size() << " waypoints "
			  << "and " << time.size() << " times." << std::endl;
	}
	else if(times_are_sound(time))
	{
		std::vector<std::vector<double>> points; points.resize(6);                         // 6 dimensions
		for(int i = 0; i < 6; i++) points[i].resize(this->n);                              // n waypoints for each dimension
		
		for(int j = 0; j < this->n; j++)
		{
			iDynTree::Position pos = waypoint[j].getPosition();                        // Get the position for the jth waypoint
			iDynTree::Vector3  rot = waypoint[j].getRotation().asRPY();                // Get the orientation as rpy angles

			for(int i = 0; i < 3; i++)
			{
				points[i][j]   = pos[i];                                           // Insert the positions
				points[i+3][j] = rot[i];                                           // Insert the rpy angles
			}
		}
		
		this->spline.resize(6);
		for(int i = 0; i < 6; i++)
		{
			this->spline[i].setData(time, iDynTree::VectorDynSize(points[i]));         // Set the data for the ith spline
		}
		
		this->isValid = true;
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Check that the times are in ascending order                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CartesianTrajectory::times_are_sound(const iDynTree::VectorDynSize &time)
{
	for(int i = 0; i < time.size()-1; i++)
	{
		if(time[i] == time[i+1] or time[i] > time[i+1])
		{
			std::cerr << "[ERROR] [CARTESIANTRAJECTORY] Constructor: "
				  << "Times are not in ascending order! "
				  << "Time " << i+1 << " was " << time[i] << " seconds and "
				  << "time " << i+2 << " was " << time[i+1] << " seconds." << std::endl;
			return false;
		}
	}
	return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Get the desired state for the given time                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CartesianTrajectory::get_state(iDynTree::Transform &pose,
			       	    iDynTree::Twist &vel,
			            iDynTree::SpatialAcc &acc,
			            const double &time)
{
	if(not this->isValid)
	{
		std::cerr << "[ERROR] [CARTESIANTRAJECTORY] get_state(): "
			  << "There was a problem during the construction of this object. "
			  << "Could not get the desired state." << std::endl;
			  
		return false;
	}
	else
	{
		iDynTree::Position pos;
		iDynTree::Vector3 rpy;
		iDynTree::GeomVector3 linearVel, angularVel, linearAcc, angularAcc;
		
		for(int i = 0; i < 3; i++)
		{
			pos[i] = this->spline[i].evaluatePoint(time, linearVel[i], linearAcc[i]);
			rpy[i] = this->spline[i+3].evaluatePoint(time, angularVel[i], angularAcc[i]);
		}
		
		pose = iDynTree::Transform(iDynTree::Rotation::RPY(rpy[0], rpy[1], rpy[2]), pos);
		vel = iDynTree::Twist(linearVel, angularVel);
		acc = iDynTree::SpatialAcc(linearAcc, angularAcc);
		
		return true;
	}
}

#endif
