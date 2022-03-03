    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                   A minimum acceleration trajectory across 2 or more points                    //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CUBIC_H_
#define CUBIC_H_

#include <Eigen/Core>                                                                              // Eigen::MatrixXd, inverse() function
#include <iDynTree/Core/MatrixDynSize.h>                                                           // iDynTree::MatrixDynSize
#include <iDynTree/Core/VectorDynSize.h>                                                           // iDynTree::VectorDynSize
#include <iostream>                                                                                // std::cerr << "lol" << std::endl;
#include <math.h>                                                                                  // pow(x,n)
#include <vector>                                                                                  // std::vector

class Cubic
{
	public:
		Cubic() {}
		
		Cubic(const std::vector<iDynTree::VectorDynSize> &point,
		      const std::vector<double> &time);
			
		bool get_state(iDynTree::VectorDynSize &pos,
			       iDynTree::VectorDynSize &vel,
			       iDynTree::VectorDynSize &acc,
			       const double &time);
		
	protected: // CubicRotation class can access these
	
		bool isValid = true;                                                               // Won't do any interpolation if this is false
		int m, n;                                                                          // m dimensions across n waypoints (for n-1 splines)
		std::vector<double> t;                                                             // Time to reach each waypoint
		std::vector<std::vector<double>> a, b, c, d;                                       // Spline coefficients
};                                                                                                 // Semicolon needed after a class declaration

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     Constructor                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
Cubic::Cubic(const std::vector<iDynTree::VectorDynSize> &point,
             const std::vector<double> &time):
             m(point[0].size()),                                                                   // Number of dimensions
             n(point.size()),                                                                      // Number of waypoints (for n-1 splines)
             t(time)                                                                               // Time to reach each waypoint
{
	// Check the inputs are sound
	if(this->n < 2)
	{
		std::cerr << "[ERROR] [CUBIC] Constructor: "
			  << "A minimum of 2 waypoints is needed to create a trajectory." << std::endl;
		this->isValid = false;
	}
	else if(this->m < 1)
	{
		std::cerr << "[ERROR] [CUBIC] Constructor: "
		          << "The input vector has zero dimensions!" << std::endl;
		this->isValid = false;
	}
	else if(point.size() != time.size())
	{
		std::cerr << "[ERROR] [CUBIC] Constructor: "
			  << "Input vectors must be the same length. The 'point' vector had " << point.size()
			  << " elements and the 'time' vector had " << time.size() << " elements." << std::endl;
		this->isValid = false;
	}
	else
	{
		for(int i = 0; i < point.size()-1; i++)
		{
			// Waypoints don't have equal dimensions
			if(point[i].size() != point[i+1].size())
			{
				std::cerr << "[ERROR] [CUBIC] Constructor: "
					  << "Point vectors are not of equal length. Point " << i << " had "
					  << point[i].size() << " elements and point " << i+1 << " had "
					  << point[i+1].size() << " elements." << std::endl;
				this->isValid = false;
			}
			// Waypoint times are equal (potential infinite velocity)
			if(time[i] == time[i+1])
			{
				std::cerr << "[ERROR] [CUBIC] Constructor: "
					  << "Cannot move in zero time! Time " << i << " was " << time[i]
					  << " seconds and time " << i+1 << " was " << time[i+1] << " seconds." << std::endl;
				this->isValid = false;
			}
			// Waypoint times are out of order
			else if(time[i] > time[i+1])
			{
				std::cerr << "[ERROR] [CUBIC] Constructor: "
					  << "Times are not in ascending order! Time " << i << " was " << time[i]
					  << " seconds and time " << i+1 << " was " << time[i+1] << " seconds." << std::endl;
				this->isValid = false;
			}
		}
	}
	
	// No problems
	if(this->isValid)
	{
		// Resize the vector of coefficients
		this->a.resize(this->m);
		this->b.resize(this->m);
		this->c.resize(this->m);
		this->d.resize(this->m);
		
		// Two waypoints
		if(this->n == 2)
		{
			double dt = time[1] - time[0];                                             // Difference in time between waypoints
			for(int i = 0; i < this->m; i++)
			{
				double dx = point[1][i] - point[0][i];                             // Difference in position between waypoints
				this->a[i].push_back( -2*dx/pow(dt,3) );
				this->b[i].push_back(  3*dx/pow(dt,2) );
				this->c[i].push_back(  0.0            );                           // Dictates start velocity of spline
				this->d[i].push_back(  point[0][i]    );                           // Dictates start position of spline
			}
			
		}
		
		// Arbitrarily many waypoints
		else
		{
			// Accelerations are related to velocities by A*xdd = B*x
			Eigen::MatrixXd A = Eigen::MatrixXd::Zero(this->n, this->n);
			Eigen::MatrixXd B = Eigen::MatrixXd::Zero(this->n, this->n);
			
			// Set the constraints for the start of the trajectory
			double dt1 = this->t[1] - this->t[0];
			double dt2 = this->t[2] - this->t[1];
			A(0,0) = dt1/2;
			A(0,1) = dt1/2 + dt2/3;
			A(0,2) = dt2/6;
//                      B(0,0) = 0.0;
			B(0,1) = -1/dt2;
			B(0,2) = 1/dt2;
			
			// Set the constraints in the middle of the trajectory
			for(int i = 1; i < this->n-2; i++)
			{
				dt1 = this->t[i] - this->t[i-1];
				dt2 = this->t[i+1] - this->t[i];
				
				A(i,i-1) = dt1/6;
				A(i,i)   = (dt1 + dt2)/3;
				A(i,i+1) = dt2/6;
				
				B(i,i-1) = 1/dt1;
				B(i,i)   = -(1/dt1 + 1/dt2);
				B(i,i+1) = 1/dt2;
			}
			
			// Set the constraints at the end of the trajectory
			double dt = this->t[this->n-1] - this->t[this->n-2];
//                      A(this->n-1,this->n-3) = 0.0;
			A(this->n-1,this->n-2) = dt/6;
			A(this->n-1,this->n-1) = dt/3;
//                      B(this->n-1,this->n-3) = 0.0;
			B(this->n-1,this->n-2) = 1/dt;
			B(this->n-1,this->n-1) =-1/dt;
			
			Eigen::MatrixXd C = A.inverse()*B;                                         // This makes things a little easier
			Eigen::VectorXd x(this->n), xddot(this->n);                                // A*xddot = B*x ---> xdd = C*x
			for(int i = 0; i < this->m; i++)
			{
				// 'point' is stored as n waypoints with m dimensions,
				// but vectors a, b, c, and d are stored as m dimensions with
				// n waypoints, hence the weird indexing
				for(int j = 0; j < this->n; j++) x(j) = point[j][i];               // Get all n waypoints of the ith dimension
				
				xddot = C*x;                                                       // Accelerations for all n waypoints
				
				for(int j = 0; i < this->n-1; j++)                                 // There are only n-1 splines
				{
					double dt = this->t[i+1] - this->t[i];
					
					a[i].push_back( (xddot(i+1) - xddot(i))/(6*dt) );
					b[i].push_back(  xddot(i)/2                    );
					
					if(i == 0)             c[i].push_back(  0.0                                            );
					else if(i < this->n-2) c[i].push_back( (x(i+1)-x(i))/dt - (xddot(i+1)+2*xddot(i))*dt/6 );
					else                   c[i].push_back(-(xddot(i) + xddot(i+1))*dt/2                    );
					
					d[i].push_back( x(i) );
				}
			}
		}
	}
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                             Get the state for the given time                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Cubic::get_state(iDynTree::VectorDynSize &pos, iDynTree::VectorDynSize &vel, iDynTree::VectorDynSize &acc, const double &time)
{
	if(not this->isValid)
	{
		std::cerr << "[ERROR] [CUBIC] get_state(): "
			  << "Something went wrong during the construction of this object. "
			  << "Cannot obtain the state." << std::endl;
		return false;
	}
	else if(pos.size() != vel.size() or vel.size() != acc.size())
	{
		std::cerr << "[ERROR] [CUBIC] get_state(): "
			  << "Vectors are not of equal length. pos had " << pos.size() << " << elements, "
			  << "vel had " << vel.size() << " elements and acc had " << acc.size() << " elements." << std::endl;
		return false;
	}
	else
	{
		if(time < this->t[0])                                                              // Not yet started
		{
//			std::cout << "Time = " << time << " seconds (not yet started)." << std::endl;
			for(int i = 0; i < this->m; i++)
			{
				pos[i] = this->d[i][0];                                            // Remain at the start
				vel[i] = 0.0;                                                      // Don't move
				acc[i] = 0.0;                                                      
			}
		}
		else if(time < this->t[this->n-1])                                                 // Somewhere in the middle
		{
			int j;
			for(int i = 1; i < this->n-1; i++)
			{
				if(time < this->t[i])                                              // Not yet started the ith spline...
				{
					j = i-1;                                                   // ... so we must be on spline i-1   
					break;
				}
			}
			
			double dt = time - this->t[j];                                             // Elapsed time since start of jth spline
			
//			std::cout << "Time = " << time << " seconds. On the spline " << j+1 << "."
//				  << "Elapsed time: " << time - this->t[j] << " seconds." << std::endl;
				  
			for(int i = 0; i < this->m; i++)
			{
				pos[i] =   this->a[i][j]*pow(dt,3) +   this->b[i][j]*pow(dt,2) + this->c[i][j]*dt + this->d[i][j];
				vel[i] = 3*this->a[i][j]*pow(dt,2) + 2*this->b[i][j]*dt        + this->c[i][j];
				acc[i] = 6*this->a[i][j]*dt        + 2*this->b[i][j];
			}
		}
		else                                                                               // Finished
		{
			double dt = this->t[this->n-1] - this->t[this->n-2];                       // Elapsed time from start of last spline to the end
			
//			std::cout << "Time = " << time << " seconds (finished). 'Elapsed' time = "
//				  << dt << " seconds." << std::endl;
			for(int i = 0; i < this->m; i++)
			{
				pos[i] = this->a[i][this->n-2]*pow(dt,3)
				       + this->b[i][this->n-2]*pow(dt,2)
				       + this->c[i][this->n-2]*dt
				       + this->d[i][this->n-2];
				vel[i] = 0.0;
				acc[i] = 0.0;
			}
		}
		return true;	
	}
}
#endif
