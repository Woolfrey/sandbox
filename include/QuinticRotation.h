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
			
	private:
		iDynTree::Rotation R0;                                                             // Initial rotation

};                                                                                                 // Semicolon needed after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       Constructor                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
QuinticRotation::QuinticRotation(const iDynTree::Rotation &startPoint,
                                 const iDynTree::Rotation &endPoint,
                                 const double &startTime,
                                 const double &endTime):
                                 R0(startPoint)
{
	// Variables in this scope
	double angle;
	double axis[3];
	double norm = 0;
	iDynTree::Rotation dR;
	iDynTree::Vector4 dQ;
	iDynTree::VectorDynSize p1(3); p1.zero();
	iDynTree::VectorDynSize p2(3);
	
	// We need to generate a trajectory across the *difference* in rotation:
	// R(t) = R(t0)*dR(t) ---> dR(tf) = R(t0)'R(tf)
	dR = startPoint.inverse()*endPoint;                                                        // Difference in rotation
	dQ =  dR.asQuaternion();                                                                   // Difference in rotation expressed as a quaternion
	
	// Get the angle encoded in the quaternion
	angle = 2*acos(dQ[0]);                                                                     // eta = cos(0.5*angle)
	if(angle > M_PI) angle = 2*M_PI - angle;                                                   // If > 180 degrees, take the shorter path
	
	// Get the axis encoded in the quaternion
	for(int i = 0; i < 3; i++) norm += dQ[i+1]*dQ[i+1];                                        // Sum of squares
	norm = sqrt(norm);                                                                         // Norm of the vector part of the quaternion
	for(int i = 0; i < 3; i++)
	{
		axis[i] = dQ[i+1]/norm;                                                            // epsilon = sin(0.5*angle)*axis
		p2[i] = angle*axis[i];
	}
	
	// Now input to the derived class
	Quintic(p1, p2, startTime, endTime);                                                       // NOTE: Start at zero, interpolate over angle*axis
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Get the desired state for the given time                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool QuinticRotation::get_state(iDynTree::Rotation &rot,
				iDynTree::GeomVector3 &vel,
				iDynTree::GeomVector3 &acc,
				const double &time)
{
	// Variables in this scope
	double angle = 0;                                                                          // Difference in angle of rotation
	double axis[3];                                                                            // Axis of rotation
	iDynTree::Rotation dR;                                                                     // Difference in rotation
	iDynTree::VectorDynSize p(3), v(3), a(3);                                                  // Position, velocity, and acceleration
	iDynTree::Vector4 dQ;                                                                      // Difference in orientation as a quaternion
	
	std::cout << "We are in the Quintic Rotation object." << std::endl;
	
	std::cout << "UNABLE TO ACCESS BASE CLASS! HAS IT BEEN SET PROPERLY?" << std::endl;
	if(Quintic::get_state(p, v, a, time))
	{
		std::cout << "State obtained from base class." << std::endl;
		
		for(int i = 0; i < 3; i++) angle += p[i]*p[i];                                     // Sum of squares
		angle = sqrt(angle);                                                               // This completes the norm
	
		dQ[0] = cos(0.5*angle);                                                            // Scalar part of the quaternion
		for(int i = 0; i < 3; i++)
		{
			axis[i] = p[i]/angle;                                                      // p = angle*axis
			dQ[i+1] = sin(0.5*angle)*axis[i];                                          // Vector part of the quaternion
			vel[i] = v[i];
			acc[i] = a[i];
		}
		dR.fromQuaternion(dQ);                                                             // Convert to rotation object
		rot = this->R0*dR;                                                                 // R(t) = R0*dR(t)
		return true;
	}
	else
	{
		std::cerr << "[ERROR] [QUINTICROTATION] get_state() : Could not obtain the state for the given time." << std::endl;
		rot = this->R0;
		for(int i = 0; i < 3; i++)
		{
			vel[i] = 0;
			acc[i] = 0;
		}
		return false;
	}
}

#endif
