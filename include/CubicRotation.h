    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                A minimum acceleration trajectory across 3 or more orientations                 //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CUBICROTATION_H_
#define CUBICROTATION_H_

#include <Cubic.h>
#include <iDynTree/Core/Rotation.h>                                                                // iDynTree::Rotation

class CubicRotation : public Cubic
{
	public:
		CubicRotation() : Cubic() {}                                                       // Empty constructor
		
		CubicRotation(const std::vector<iDynTree::Rotation> &rots,                         // Proper constructor
			      const std::vector<double> &times);
			      
		bool get_state(iDynTree::Rotation &rot,                                            // Get the desired state for the current time
			       iDynTree::GeomVector3 &vel,
			       iDynTree::GeomVector3 &acc,
			       const double &time)
			       
	private:
		iDynTree::Rotation R0;                                                             // Initial rotation
			       
};                                                                                                 // Semicolon needed after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
CubicRotation::CubicRotation(const std::vector<iDynTree::Rotation> &rots,
			     const std::vector<double> &times):
			     Cubic(),                                                              // Base object
			     R0(rots[0])		
{
	// Set the values in the base class
	this->m = 3;                                                                               // 3 dimensions for RPY
	this->n = rots.size();                                                                     // Number of waypoints
	
	// Check the inputs are sound
	if(this->n < 2)
	{
		std::cerr << "[ERROR] [CUBICROTATION] Constructor: "
			  << "A minimum of 2 waypoints is needed to create a trajectory." << std::endl;
		this->isValid = false;
	}
	else if(this->m < 1)
	{
		std::cerr << "[ERROR] [CUBICROTATION] Constructor: "
		          << "The input vector has zero dimensions!" << std::endl;
		this->isValid = false;
	}
	else if(rots.size() != times.size())
	{
		std::cerr << "[ERROR][CUBICROTATION] Constructor: "
			  << "Input vectors are not of equal length. "
			  << "There were " << rots.size() << " rotations and "
			  << times.size() << " times." <<std::endl;
		this->isValid = false;
	}
	else
	{
		// Convert every iDynTree::Rotation object to rpy
		std::vector<iDynTree::VectorDynSize> points;
		for(int i = 0; i < rots.size(); i++)
		{
			points.push_back(rots[i].asRPY());                                         // Convert to RPY
			
			// Check the times are in ascending order
			if(i > 0)
			{
				if(times[i-1] == times[i])
				{
					std::cerr << "[ERROR] [CUBICROTATION] Constructor: " 
						  << "Cannot move in zero time! "
						  << "Time " << i - 1 << " of " << times[i-1] << " seconds was the same as "
						  << "time " << i << " of " << times[i] << " seconds." << std::endl;
					this->isValid = false;
				}
				else if(times[i-1] == times[i])
				{
					std::cerr << "[ERROR] [CUBICROTATION] Constructor: " 
						  << "Times are not in ascending order! "
						  << "Time " << i - 1 << " of " << times[i-1] << " seconds was greater than "
						  << "time " << i << " of " << times[i] << " seconds." << std::endl;
					this->isValid = false;
				}
			}
		}
		
		if(this->isValid()) compute_coefficients(points, times);
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the desired state for the given time                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CubicRotation::get_state(iDynTree::Rotation &rot, iDynTree::GeomVector3 &vel, iDynTree::GeomVector3 &acc, const double &time)
{
	iDynTree::VectorDynSize p(3), v(3), a(3);                                                  // Base class works with VectorDynSize
	
	if(Cubic::get_state(p,v,a,time))                                                           // Call the get_state() function from the base class
	{
		rot = iDynTree::Rotation::RPY(p[0], p[1], p[2]);
		
		for(int i = 0; i < 3; i++)
		{
			vel[i] = v[i];
			acc[i] = a[i];
		}
		
		return true;
	}
	else
	{
		std::cout << "[ERROR] [CUBICROTATION] get_state(): "
			  << " There was a problem obtaining the desired state." << std::endl;
			  
		rot = this->R0;                                                                    // Remain at the start
		vel.zero(); acc.zero();                                                            // Don't move
		
		return false;
	}
}

#endif
