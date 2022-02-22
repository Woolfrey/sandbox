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
			
		bool get_state(iDynTree::Rotation &rot,
			       iDynTree::GeomVector3 &vel,
			       iDynTree::GeomVector3 &acc,
			       const double &time);
	
	private:
		std::vector<iDynTree::Rotation> R;                                                 // Vector of waypoints

};                                                                                                 // Semicolon needed after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Constructor                                                      //
////////////////////////////////////////////////////////////////////////////////////////////////////
CubicRotation::CubicRotation(const std::vector<iDynTree::Rotation> &rots,
			     const std::vector<double> &times):
			     Cubic(),                                                              // Base object		
			     R(rots)
{
	// Need to set these values in the base class
	this->t = times; this->m = 3; this->n = rots.size();
	
	if(rots.size() != times.size())
	{
		std::cerr << "[ERROR] [CUBICROTATION] Constructor : Input vectors are not of equal length." << std::endl;
		std::cerr << "rots: " << rots.size() << " times " << times.size() << std::endl;
	}
	else
	{
		this->isNotValid = false;
		
		for(int i = 0; i < this->n-1; i++)
		{
			if(times[i] > times[i+1])
			{
				std::cerr << "[ERROR] [CUBICROTATION] Constructor : Times are not in ascending order." << std::endl;
				this->isNotValid = false;
				break;
			}
		}
		
		// Variables used in this scope
		double angle;                                                                      // Angle between 2 waypoints
		iDynTree::Vector3 axis;                                                            // Axis of rotation for said angle
		iDynTree::Vector4 dQ;                                                              // Difference in orientation as quaternion
		std::vector<iDynTree::VectorDynSize> points; points.resize(this->n);
		
		// 1. Solve all differences in rotations: dR(i) = R(i)'*R(i+1)
		for(int i = 0; i < this->n-1; i++)
		{
			dQ = (rots[i].inverse()*rots[i+1]).asQuaternion();                         // Get the difference between orientations
			
			// Get the angle
			angle = 2*acos(dQ[0]);                                                     // eta = cos(0.5*angle)
			if(angle > M_PI) angle = 2*M_PI - angle;                                   // If greater than 180 degrees, take the shorter path
			
			// Get the axis
			double norm = 0;
			for(int i = 0; i < 3; i++) norm += dQ[i+1]*dQ[i+1];                        // Sum of squares
			norm = sqrt(norm);                                                         // Complete the norm
			for(int i = 0; i < 3; i++) axis[i] = dQ[i+1]/norm;                         //  epsilon = sin(0.5*angle)*axis
			
			// Put it in to the vector object as angle*axis
			points[i].resize(3);
			for(int i = 0; i < 3; i++) points[i] = angle*axis[i];                      // Difference in orientation as angle*axis
		}
		points[this->n-1].zero();                                                          // Not actually used; assume zero
		
		// Resize the columns of the coefficient arrays
		this->a.resize(this->m);
		this->b.resize(this->m);
		this->c.resize(this->m);
		this->d.resize(this->m);

		// 2 waypoints, only 1 spline
		if(this->n == 2)                                                                   // n - 1 = 1 spline
		{	
			float dt = this->t[1] - this->t[0];
			for(int i = 0; i < this->m; i++)
			{
				// Note: points has n vectors with m dimensions, whereas
				// a, b, c, d, are stored as m dimensions across n waypoints
				a[i][0] = -2*(points[1][i] - points[0][i])/pow(dt,3);              // Determines the shape of position profile
				b[i][0] =  3*(points[1][i] - points[0][i])/pow(dt,2);              // Determines the shape of velocity profile
				c[i][0] = 0;                                                       // Start velocity
				d[i][0] = points[0][i];                                            // Start position
			}
		}
		// More than 1 waypoint, need to enforce continuity across waypoints
		else
		{
			// Accelerations are related to positions via A*sdd = B*s
			iDynTree::MatrixDynSize A(this->n, this->n); A.zero();
			iDynTree::MatrixDynSize B(this->n, this->n); B.zero();
			
			// Set constraints for the start and end of trajectory
			A(0,0) = (this->t[1] - this->t[0])/3;
			A(0,1) = (this->t[1] - this->t[0])/6;
			B(0,0) = 1/(this->t[1] - this->t[0]);

			// Set the constraints for the end of the trajectory
			A(this->n-1, this->n-2) = (this->t[this->n-1] - this->t[n-2])/6;
			A(this->n-1, this->n-1) = (this->t[this->n-1] - this->t[n-2])/3;
			B(this->n-1, this->n-2) = -1/(this->t[this->n-1] - this->t[this->n-2]);
			
			// Set the constraints for each waypoint inbetween
			float dt1, dt2;
			for(int i = 1; i < this->n-2; i++)
			{
				dt1 = this->t[i] - this->t[i-1];
				dt2 = this->t[i+1] - this->t[i];
				
				A(i,i-1)  = dt1/6;
				A(i,i)	  = (dt1 + dt2)/3;
				A(i, i+1) = dt2/6;
				
				B(i,i-1) = -1/dt1;
				B(i,i)   = 1/dt2;
			}
			
//			iDynTree::MatrixDynSize C = A.inverse()*B;                                 // Makes calcs a little easier
			iDynTree::VectorDynSize sdd(this->n), s(this->n);	                   // A*sdd = B*s ---> sdd = C*s
			double ds, dt;
			for(int i = 0; i < this->m; i++)
			{
				// NOTE: points is stores as n waypoints each with m dimensions
				// whereas a, b, c, d are m dimensions across n waypoints
				for(int j = 0; j < this->n; j++) s[j] = points[j][i];              // Get the jth waypoint for the ith dimension
//				sdd = C*s;                                                         // Compute the coefficients
				
				for(int j = 0; j < this->n-1; j++)                                 // Compute coefficients for the n-1 splines
				{
					dt = this->t[j+1] - this->t[j];
							
					this->a[i].push_back((sdd[j+1] - sdd[j])/(6*dt));
					this->b[i].push_back((sdd[j]/2));			
					if(j == 0)		this->c[i].push_back(0);           // Starting velocity of zero for first spline
					else if(j < this->n-1)	this->c[i].push_back(s[j]/dt - dt*(sdd[j+1] - 2*sdd[j])/6);
					else			this->c[i].push_back(dt*(sdd[j+1] + sdd[j])/2);
					this->d[i].push_back(0);                                   // d dictates the start position of the spline
				}
			}
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                           Get the desired state for the given time                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool CubicRotation::get_state(iDynTree::Rotation &rot,
			      iDynTree::GeomVector3 &vel,
			      iDynTree::GeomVector3 &acc,
			      const double &time)
{
	// Variables used in this scope
	double angle = 0;
	double axis[3];
	iDynTree::Rotation dR;
	iDynTree::VectorDynSize p(3), v(3), a(3);
	iDynTree::Vector4 dQ;
	
	// Get the desired state and convert to rotation
	if(Cubic::get_state(p, v, a, time))
	{
		for(int i = 0; i < 3; i++) angle += p[i]*p[i];                                     // Sum of squares
		angle = sqrt(angle);                                                               // Complete the norm to get the angle
		
		dQ[0] = cos(0.5*angle);                                                            // Scalar component of quaternion

		for(int i = 0; i < 3; i++)
		{
			axis[i] = p[i]/angle;                                                      // Extract the axis
			dQ[i+1] = sin(0.5*angle)*axis[i];                                          // Vector component of quaternion
			
			vel[i] = v[i]; acc[i] = a[i];                                              // Transfer vel, acc data while we're in a loop
		}
		
		dR = iDynTree::Rotation::RotationFromQuaternion(dQ);                               // Convert quaternion to rotation object
		
		// Figure out where we are on the trajectory
		int j;
		if(time < this->t[0]) j = 0;                                                       // On the first spline
		else if(time < this->t[this->n])                                                   // Somewhere in the middle
		{
			for(int i = 1; i < this->n; i++)
			{
				if(time < this->t[i])                                              // Not yet reached the ith waypoint...
				{
					j = i-1;                                                   // ... must be on spline i-1
					break;
				}
			}
		}
		else j = this->n-1;                                                                // Last spline
		
		// Insert the desired state
		rot = this->R[j]*dR;                                                               // R(t) = R(0)*dR(t)
//		vel = v;
//		acc = a;

		return true;
	}
	
	// Failed, so don't move
	else
	{
		std::cerr << "[ERROR] [CUBICROTATION] get_state() : Could not obtain the desired state." << std::endl;
		
		rot = this->R[0];                                                                  // Remain at the start
		vel.zero(); acc.zero();                                                            // Don't move
		
		return false;
	}
}

#endif
