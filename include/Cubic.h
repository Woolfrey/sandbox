/*
*	A minimum acceleration trajectory across two or more points.
*/

#ifndef CUBIC_H_
#define CUBIC_H_

#include <iDynTree/Core/MatrixDynSize.h>						// iDynTree::MatrixDynSize
#include <iDynTree/Core/VectorDynSize.h>						// iDynTree::VectorDynSize
#include <iostream>									// std::cerr << "lol" << std::endl;
#include <math.h>									// pow(x,n)
#include <vector>									// std::vector

class Cubic
{
	public:
		Cubic() {}
		
		Cubic(const std::vector<iDynTree::VectorDynSize> &points,
			const std::vector<double> &times);
			
		bool get_state(iDynTree::VectorDynSize &pos,
				iDynTree::VectorDynSize &vel,
				iDynTree::VectorDynSize &acc,
				const double &time);
		
	protected: // CubicRotation class can access these
	
		bool isNotValid = true;							// Won't do any interpolation if this is true
		int m, n;								// m dimensions across n waypoints (for n-1 splines)
		std::vector<double> t;							// Time to reach each waypoint
		std::vector<std::vector<double>> a, b, c, d;				// Spline coefficients
};											// Semicolon needed after a class declaration

/******************** Constructor ********************/
Cubic::Cubic(const std::vector<iDynTree::VectorDynSize> &points, const std::vector<double> &times)
		: m(points[0].size())							// Number of dimensions
		, n(points.size())							// Number of waypoints
		, t(times)
{
	// Check that the input dimensions are sound
	if(points.size() != times.size())
	{
		std::cerr << "[ERROR][CUBIC] Constructor : Inputs are not of equal length!"
			<< " points: " << points.size() << " times: " << times.size() << std::endl;
		this->isNotValid = true;
	}
	else
	{
		this->isNotValid = false;
		
		// Check that the times are in ascending order
		for(int i = 0; i < n-1; i++)
		{
			if(times[i] > times[i+1])
			{
				std::cerr << "[ERROR][CUBIC] Constructor : Times are not in ascending order!" << std::endl;
				this->isNotValid = true;
			}
		}
		
		// Resize the columns of the coefficient arrays
		this->a.resize(this->m);
		this->b.resize(this->m);
		this->c.resize(this->m);
		this->d.resize(this->m);
	
		// 2 waypoints, only 1 spline
		if(this->n == 2)							// n - 1 = 1 spline
		{	
			float dt = this->t[1] - this->t[0];
			for(int i = 0; i < this->m; i++)
			{
				// Note: points has n vectors with m dimensions, whereas
				// a, b, c, d, are stored as m dimensions across n waypoints
				a[i].push_back(-2*(points[1][i] - points[0][i])/pow(dt,3));
				b[i].push_back(3*(points[1][i] - points[0][i])/pow(dt,2));
				c[i].push_back(0);						// Start velocity
				d[i].push_back(points[0][i]);					// Start position
			}
		}
		// More than 1 waypoint, need to enforce continuity across waypoints
		else
		{
			// Accelerations are related to positions via A*sdd = B*s
			iDynTree::MatrixDynSize A(this->n, this->n); A.zero();
			iDynTree::MatrixDynSize B(this->n, this->n); B.zero();
			
			// Constraints on the start of the trajectory
			A(0,0) = (this->t[1] - this->t[0])/2;
			A(0,1) = (this->t[1] - this->t[0])/2 + (this->t[2] - this->t[1])/3;
			A(0,2) = (this->t[2] - this->t[1])/6;	
			B(0,1) = -1/(this->t[2] - this->t[1]);
			B(0,2) =  1/(this->t[2] - this->t[1]);
			
			// Constraints on the end of the trajectory
			A(this->n-1,this->n-2) = (this->t[this->n-1] - this->t[n-2])/6;
			A(this->n-1,this->n-1) = (this->t[this->n-1] - this->t[n-2])/3;
			B(this->n-1,this->n-2) = 1/(this->t[this->n-1] - this->t[this->n-2]);
			B(this->n-1,this->n-1) = -1/(this->t[this->n-1] - this->t[this->n-2]);
			
			// Set the constraints for each waypoint inbetween
			double dt1, dt2;
			for(int i = 1; i < this->n-2; i++)
			{
				dt1 = this->t[i] - this->t[i-1];
				dt2 = this->t[i+1] - this->t[i];
				
				A(i,i-1) = dt1/6;
				A(i,i)	 = (dt1 + dt2)/3;
				A(i,i+1) = dt2/6;
				
				B(i,i-1) = 1/dt1;
				B(i,i)	 = -1/dt1 - 1/dt2;
				B(i,i+1) = 1/dt2;
			}
			
//			iDynTree::MatrixDynSize C = A.inverse()*B;				// Makes calcs a little easier
			iDynTree::VectorDynSize sdd(this->n), s(this->n);			// A*sdd = B*s ---> sdd = C*s
			double ds, dt;
			for(int i = 0; i < this->m; i++)
			{
				// NOTE: points is stores as n waypoints each with m dimensions
				// whereas a, b, c, d are m dimensions across n waypoints
				for(int j = 0; j < this->n; j++) s[j] = points[j][i];		// Get the jth waypoint for the ith dimension
//				sdd = C*s;							// Compute the coefficients
				
				for(int j = 0; j < this->n-1; j++)				// Compute coefficients for the n-1 splines
				{
					dt = this->t[j+1] - this->t[j];
					ds = s[j+1] - s[j];
							
					this->a[i].push_back((sdd[j+1] - sdd[j])/(6*dt));
					this->b[i].push_back((sdd[j]/2));			
					if(j == 0)		this->c[i].push_back(0);	// Starting velocity of zero for first spline
					else if(j < this->n-1)	this->c[i].push_back(ds/dt - dt*(sdd[j+1] - 2*sdd[j])/6);
					else			this->c[i].push_back(dt*(sdd[j+1] + sdd[j])/2);
					this->d[i].push_back(s[j]);				// d dictates the start position of the spline
				}
			}
		}
	}
}

/******************** Get the desired state for the given time ********************/
bool Cubic::get_state(iDynTree::VectorDynSize &pos, iDynTree::VectorDynSize &vel, iDynTree::VectorDynSize &acc, const double &time)
{
	// Check that the input arguments are the same length
	if(pos.size() != vel.size() || vel.size() != acc.size())
	{
		std::cerr << "[ERROR][CUBIC] get_state() : Vectors are not of equal length!"
			<< " pos: " << pos.size() << " vel: " << vel.size() << " acc: " << acc.size() << std::endl;
		
		pos.resize(this->m); vel.resize(this->m); acc.resize(this->m);
		for(int i = 0; i < this->m; i++)
		{
			pos[i] = this->d[i][0];							// Remain at start
			vel[i] = 0.0; acc[i] = 0.0;						// Don't move
		}
		return false;
	}
	
	// Check if the object is valid upon construction
	else if(this->isNotValid)								// Something wrong with this object
	{
		std::cerr << "[ERROR][CUBIC] get_state() : Something went wrong during construction of this object. "
			<< "Cannot return the state." << std::endl;
				
		for(int i = 0; i < this->m; i++)
		{
			pos[i] = this->d[i][0];							// Remain at the start
			vel[i] = 0.0; acc[i] = 0.0;						// Don't move
		}
		return false;
	}
	
	else
	{
		// Trajectory not yet started
		if(time < this->t[0])
		{
			for(int i = 0; i < this->m; i++)
			{
				pos[i] = this->d[i][0];						// d coefficient dictates start position
				vel[i] = 0.0; 							// Don't move
				acc[i] = 0.0;							// Don't accelerate
			}
		}
		// Trajectory finished
		else if(time > this->t.back())
		{
			int j = this->n-1;							// Start from spline n-1
			double dt = this->t[n] - this->t[n-1];					// Interpolate time up to the last waypoint
			for(int i = 0; i < this->m; i++)
			{
				pos[i] = this->a[i][j]*pow(dt,3) + this->b[i][j]*pow(dt,2) + this->c[i][j]*dt + this->d[i][j];
				vel[i] = 0.0;							// Don't move
				acc[i] = 0.0;							// Don't accelerate
			}
		}
		// Somewhere in between
		else
		{
			// Figure out which spline we are on
			int j; double dt;
			for(int i = 1; i < this->n; i++)
			{
				if(time < this->t[i])						// Not yet reached the ith waypoint...
				{
					j = i-1;						// ... so we must be on spline i-1
					dt = time - this->t[j];					// Elapsed time since start of spline i-1
				}
			}
		
			// Interpolate the state
			for(int i = 0; i < this->m; i++)
			{
				pos[i] =   this->a[i][j]*pow(dt,3) +   this->b[i][j]*pow(dt,2) + this->c[i][j]*dt + this->d[i][j];
				vel[i] = 3*this->a[i][j]*pow(dt,2) + 2*this->b[i][j]*dt        + this->c[i][j];
				acc[i] = 6*this->a[i][j]*dt 	   + 2*this->b[i][j];
			}
		}
		return true;
	}
}

#endif
