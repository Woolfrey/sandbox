    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //          A trajectory across two or more poses (position & orientation) in 3D space.           //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CARTESIANTRAJECTORY_H_
#define CARTESIANTRAJECTORY_H_

#include <iDynTree/Core/CubicSpline.h>
#include <iDynTree/Core/SpatialAcc.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>

class CartesianTrajectory
{
	public:
		CartesianTrajectory ();
		
		CartesianTrajectory(const std::vector<iDynTree::Transform> &waypoint,
				    const std::vector<double> &time);
				    
		bool get_state(iDynTree::Transform &pose,
			       iDynTree::Twist &vel,
			       iDynTree::SpatialAcc &acc,
			       const double &time);
	private:
		bool isValid = true;                                                               // Won't do computations if this is false
		int n;                                                                             // Number of waypoints
		std::vector<iDynTree::CubicSpline> spline;                                         // Array of splines
	
};                                                                                                 // Semicolon needed after class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                         Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
CartesianTrajectory::CartesianTrajectory(const std::vector<iDynTree::Transform> &waypoint,
				    	 const std::vector<double> &time)
{
	// Check the inputs are sound
	if(waypoint.size() != time.size())
	{
		std::cerr << "[ERROR] [CARTESIANTRAJECTORY] Constructor: "
			  << "Vectors were not of equal length! "
			  << "There were " << waypoint.size() << " waypoints and "
			  << time.size() << " times." << std::endl;
		this->isValid = false;
	}
	else
	{
		// Check the times are in order
		for(int i = 0; i < this->n-1; i++)
		{
			if(time[i] == time[i+1])
			{
				std::cerr << "[ERROR] [CARTESIANTRAJECTORY] Constructor: "
					  << "Cannot move in zero time! "
					  << "Time " << i+1 << " was " << time[i] << " seconds "
					  << "and time " << i+2 << " was " << time[i+1] << " seconds." << std::endl;
					  
				this->isValid = false;
				break;
			}
			else if(time[i] > time[i+1])
			{
				std::cerr << "[ERROR] [CARTESIANTRAJECTORY] Constructor: "
					  << "Times are not in ascending order! "
					  << "Time " << i+1 << " was " << time[i] << " seconds "
					  << "and time " << i+2 << " was " << time[i+1] << " seconds." << std::endl;
					  
				this->isValid = false;
				break;
			}
		}
		
		if(this->isValid)
		{
			std::vector<iDynTree::VectorDynSize> dim;                                   // Array of all positions and orientations
			for(int i = 0; i < 6; i++) dim.push_back(iDynTree::VectorDynSize(this->n)); // Put an nx1 vector in each of the 6 dimensions
			
			iDynTree::VectorDynSize t(this->n);                                        // Vector of times for each waypoint
			
			for(int j = 0; j < this->n; j++)
			{
				t[j] = time[j];                                                    // Add time to the vector
				
				iDynTree::Position pos = waypoint[j].getPosition();                // Get the position for the jth waypoint
				iDynTree::Vector3 rpy = waypoint[j].getRotation().asRPY();         // Get the RPY angles for the jth waypoint
				
				for(int i = 0; i < 3; i++)
				{
					dim[i][j]   = pos[i];                                      // Append position
					dim[i+1][j] = rpy[i];                                      // Append rpy angles
				}
			}
			
			for(int i = 0; i < 6; i++)
			{
				// Try and create the CubicSpline object
				if(not this->spline[i].setData(t, dim[i]))
				{
					std::cerr << "[ERROR] [CARTESIANTRAJECTORY] Constructor: "
						  << "Could not create a spline for the dim " << i+1 << "." << std::endl;
					this->isValid = false;
					break;
				}
				else
				{
					this->spline[i].setInitialConditions(0.0, 0.0);            // Start from rest
					this->spline[i].setFinalConditions(0.0, 0.0);              // End at rest
				}
			}
		}	
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Get the desired state for the given time                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CartesianTrajectory::get_state(iDynTree::Transform &pose,
			       	    iDynTree::Twist &vel,
			            iDynTree::SpatialAcc &acc,
			            const double &time)
{
	iDynTree::Vector3 p, rpy;
	iDynTree::GeomVector3 linearVel, angularVel, linearAcc, angularAcc; 
	for(int i = 0; i < 3; i++)
	{
		p[i]   = this->spline[i].evaluatePoint(time, linearVel[i], linearAcc[i]);          // Evaluate the linear component
		rpy[i] = this->spline[i].evaluatePoint(time, angularVel[i], angularAcc[i]);         // Evaluate the angular component
	}
	
	pose.setPosition(iDynTree::Position(p[0], p[1], p[2]));                                    // Set the translation component
	pose.setRotation(iDynTree::Rotation::RPY(p[3], p[4], p[5]));                               // Set the rotation component
	vel = iDynTree::Twist(linearVel, angularVel);                                              // Set the velocities
	acc = iDynTree::SpatialAcc(linearAcc, angularAcc);                                         // Set the accelerations
	
	return true;
}
#endif
