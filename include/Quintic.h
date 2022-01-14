#ifndef QUINTIC_H_
#define QUINTIC_H_

#include <iostream>									// std::cin, std::cout, std::cerr
#include <iDynTree/Core/VectorDynSize.h>						// iDynTree::VectorDynSize
#include <math.h>									// pow(x,n)

class Quintic
{
	public:
		Quintic() {}
		
		Quintic(const iDynTree::VectorDynSize &startPoint,
			const iDynTree::VectorDynSize &endPoint,
			const double &startTime,
			const double &endTime);
	
		bool get_state(iDynTree::VectorDynSize &pos,
				iDynTree::VectorDynSize &vel,
				iDynTree::VectorDynSize &acc,
				const double &time);
	
	private:
		bool isNotValid = false;						// Will not do any computations if there is a problem
		double a, b, c;								// Polynomial coefficients
		double t1, t2;								// Start time and end time for the trajectory
		iDynTree::VectorDynSize p1, p2;						// Start point and end point
		int m;									// Number of dimensions
		
};											// Semicolon needed after class declaration

/******************** Constructor ********************/
Quintic::Quintic(const iDynTree::VectorDynSize &startPoint,
		const iDynTree::VectorDynSize &endPoint,
		const double &startTime,
		const double &endTime)
		: p1(startPoint)
		, p2(endPoint)
		, t1(startTime)
		, t2(endTime)
		, m(startPoint.size())
{
	// Check the times are in order
	if(this->t1 > this->t2)
	{
		std::cerr 	<< "[WARNING][QUINTIC] Constructor : Start time of "
				<< this->t1 << " is greater than end time of " << this->t2
				<< ". Swapping the values..." << std::endl;
				
		double temp = this->t1;
		this->t1 = this->t2;
		this->t2 = temp;
	}
	if(startPoint.size() != endPoint.size())
	{
		std::cerr 	<< "[ERROR][QUINTIC] Constructor : Input vectors are not of equal length!"
				<< " Start point has " << startPoint.size() << " elements and end point has "
				<< endPoint.size() << " elements." << std::endl;
		this->isNotValid = true;
	}
	else
	{
		// Compute the the coefficients for x(t) = a*t^5 + b*t^4 + c*t^3
		double dt = this->t2 - this->t1;
		this->a =  6*pow(dt,-5);
		this->b =-15*pow(dt,-4);
		this->c = 10*pow(dt,-3);
	}	
}

/******************** Return the desired state for the given time ********************/
bool Quintic::get_state(iDynTree::VectorDynSize &pos, iDynTree::VectorDynSize &vel, iDynTree::VectorDynSize &acc, const double &time)
{
	if(pos.size() != vel.size() || vel.size() != acc.size())
	{
		std::cerr	<< "[ERROR][QUINTIC] get_state() : Input vectors are not of equal length!"
				<< " pos: " << pos.size() << " vel: " << vel.size() << " acc: " << acc.size() << std::endl;
		pos = this->p1;
		vel.resize(this->m); vel.zero();
		acc.resize(this->m); acc.zero();
		return false;
	}
	else if(this->isNotValid)
	{
		std::cerr 	<< "[ERROR][QUINTIC] get_state() : Could not obtain the desired state from this object. "
				<< "Check that it was constructed correctly." << std::endl;
				
		pos = this->p1;									// Remain at the start
		vel.zero(); acc.zero();								// Don't move
		return false;
	}
	else
	{
		// Determine the elapsed time
		double dt;
		if(time < this->t1) 		dt = 0.0;					// Remain at start
		else if(time < this->t2) 	dt = time - this->t1;				// Somewhere in the middle
		else				dt = this->t2 - this->t1;			// Remain at end
		
		// Compute the coefficients for interpolating along the trajectory
		double s =  	this->a*pow(dt,5) +    this->b*pow(dt,4) +   this->c*pow(dt,3);
		double sd =   5*this->a*pow(dt,4) +  4*this->b*pow(dt,3) + 3*this->c*pow(dt,2);
		double sdd = 20*this->a*pow(dt,3) + 12*this->b*pow(dt,2) + 6*this->c*dt;
		
		// Compute the desired state along the trajectory
		for(int i = 0; i < this->m; i++)
		{
			pos[i] = (1-s)*this->p1[i] + s*this->p2[i];
			vel[i] =  sd*(this->p2[i] - this->p1[i]);
			acc[i] = sdd*(this->p2[i] - this->p1[i]);
		}
		return true;
	}
}

#endif
