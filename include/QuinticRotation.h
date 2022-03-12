    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                A minimum jerk trajectory between orientations points in space                  //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef QUINTICROTATION_H_
#define QUINTICROTATION_H_

#include <iDynTree/Core/Rotation.h>                                                                // iDynTree::Rotation
#include <iDynTree/Core/VectorFixSize.h>                                                           // iDynTree::Vector4 (quaternion)
#include <Quintic.h>

class QuinticRotation : public Quintic
{
	public:
		QuinticRotation() : Quintic() {}                                                   // Empty constructor
		
		QuinticRotation(const iDynTree::Rotation &startPoint,
				const iDynTree::Rotation &endPoint,
				const double &startTime,
				const double &endTime);
		
		bool get_state(iDynTree::Rotation &rot,
				iDynTree::GeomVector3 &vel,
				iDynTree::GeomVector3 &acc,
				const double &time);
};                                                                                                 // Semicolon needed after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
QuinticRotation::QuinticRotation(const iDynTree::Rotation &startPoint,
                                 const iDynTree::Rotation &endPoint,
                                 const double &startTime,
                                 const double &endTime)
{
	// Set these values in the base class
	this->t1 = startTime;
	this->t2 = endTime;
	this->m = 3;
	
	// Check the inputs are sound
	if(startTime == endTime)
	{
		std::cerr << "[ERROR] [QUINTICROTATION] Constructor: "
			  << "Start time of " << startTime << " is equal to end time of " << endTime << "." << std::endl;
		this->isValid = false;
	}
	else
	{
		if(startTime > endTime)
		{
			std::cout << "[WARNING] [QUINTICROTATION] Constructor: "
				  << "Start time of " << startTime << " is greater than end time of " << endTime << ". "
				  << "Swapping their values to avoid problems..." << std::endl;
				  
			this->t1 = endTime;
			this->t2 = startTime;
		}
		
		iDynTree::Vector3 temp1 = startPoint.asRPY();
		iDynTree::Vector3 temp2 = endPoint.asRPY();
		
		// Put the rpy values in to the 'position' vectors in the base class
		this->p1.resize(3); this->p2.resize(3);
		for(int i = 0; i < 3; i++)
		{
			this->p1[i] = temp1[i];
			this->p2[i] = temp2[i];
		}
		
		// Compute the the coefficients for x(t) = a*t^5 + b*t^4 + c*t^3
		double dt = this->t2 - this->t1;
		this->a =  6*pow(dt,-5);
		this->b =-15*pow(dt,-4);
		this->c = 10*pow(dt,-3);
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Get the desired state for the given time                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool QuinticRotation::get_state(iDynTree::Rotation &rot,
				iDynTree::GeomVector3 &vel,
				iDynTree::GeomVector3 &acc,
				const double &time)
{
	iDynTree::VectorDynSize p(3), v(3), a(3);
	if(Quintic::get_state(p,v,a,time))
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
		std::cerr << "[ERROR] [QUINTICROTATION] get_state(): "
			  << "Could not obtain the state." << std::endl;
			  
		rot = iDynTree::Rotation::RPY(this->p1[0], this->p1[1], this->p1[2]);
		vel.zero();
		acc.zero();
		
		return false;
	}
}

#endif
