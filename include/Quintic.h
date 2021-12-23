#ifndef QUINTIC_H_
#define QUINTIC_H_
#include <math.h>
#include <yarp/os/LogStream.h>							// yarp::Info() and the like
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>


class Quintic
{
	public:
		// Constructors
		Quintic() {}							// Empty constructor
		
		Quintic(const yarp::sig::Vector &startPoint,			// Trajectory between two points
			const yarp::sig::Vector &endPoint,
			const float &startTime,
			const float &endTime);
		
		Quintic(const yarp::sig::Matrix &startRotation,
			const yarp::sig::Matrix &endRotation,
			const float &startTime,
			const float &endTIme);
			
		// Get Functions
		void get_state(yarp::sig::Vector &pos,				// Get desired state for position trajectory
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &time);
		
		void get_state(yarp::sig::Matrix &rot,				// Get desired state for orientation trajectory
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &time);
	
	private:
		bool isRotation;
		float a, b, c;							// Polynomial coefficients
		float s, sd, sdd;						// Interpolation coefficients
		float t1, t2;							// Start time and end time
		yarp::sig::Vector p1, p2;					// Start point and end point
		yarp::sig::Vector axisAngle;
		yarp::sig::Matrix R1;
		
		void compute_coefficients();
		void compute_scalars(const float &time);
		
};										// Semicolon needed after a class declaration

/******************** Constructor for position trajectory ****************************************/
Quintic::Quintic(const yarp::sig::Vector &startPoint,				// Trajectory between two points
		const yarp::sig::Vector &endPoint,
		const float &startTime,
		const float &endTime)
		:
		isRotation(false),
		p1(startPoint),
		p2(endPoint),
		t1(startTime),
		t2(endTime)
{
	// Check the inputs are sound
	if(startPoint.size() != endPoint.size())
	{
		yError() << "Quintic::Quintic() : Inputs vectors are not of equal length! The start point has" << this->p1.size()
			<< "elements and the end point has" << this->p2.size() << "elements.";
	}
	if(startTime > endTime)
	{
		yError() << "Quintic::Quintic() : Start time" << this->t1 << "is greater than end time" << this->t2 << "! Swapping the values...";
		float temp = this->t2;
		this->t2 = this->t1;
		this->t1 = temp;
	}
	compute_coefficients();
}

/******************** Constructor for an orientation trajectory ****************************************/
Quintic::Quintic(const yarp::sig::Matrix &startRotation,
		const yarp::sig::Matrix &endRotation,
		const float &startTime,
		const float &endTime)
		:
		isRotation(true),
		R1(startRotation),
		t1(startTime),
		t2(endTime)
{
	if(startRotation.rows() != 3
	|| startRotation.cols() != 3
	|| endRotation.rows() != 3
	|| endRotation.cols() != 3)
	{
		yError() << "Quintic::Quintic:: Expected SO3 matrices for the inputs. Start rotation is"
			<< startRotation.rows() << "x" << startRotation.cols() << "and end rotation is"
			<< endRotation.rows() << "x" << endRotation.cols();
	}
	if(startTime > endTime)
	{
		yError() << "Quintic::Quintic() : Start time" << this->t1 << "is greater than end time" << this->t2 << "! Swapping the values...";
		float temp = this->t2;
		this->t2 = this->t1;
		this->t1 = temp;
	}
	compute_coefficients();

	// For orientation, we need to interpolate over the *difference* such that:
	// R1*dR = R2 ---> dR = inv(R1)*R2
	yarp::sig::Matrix dR = startRotation.transposed()*endRotation;		// Get the difference in rotation
	this->axisAngle = yarp::math::dcm2axis(dR);				// Convert to axis-angle representation

	if(axisAngle[3] > M_PI)							// If rotation angle > 180 degrees...
	{	
		axisAngle[3] = 2*M_PI - axisAngle[3];				// ... take the shorter route
		//for(int i = 0; i < 3; i++) axisAngle[i] *= -1;			// ... and flip the axis to match?
	}
}

/******************** Get the desired position for the given time ****************************************/
void Quintic::get_state(yarp::sig::Vector &pos,					// Get desired state for position trajectory
			yarp::sig::Vector &vel,
			yarp::sig::Vector &acc,
			const float &time)
{
	// Check the inputs are sound
	if(pos.size() != vel.size() || vel.size() != acc.size())
	{
		yError() << "Quintic::get_state() : Vectors are not of equal length!"
			<< "Position has" << pos.size() << "elements, velocity has" << vel.size()
			<< "elements and acceleration has" << acc.size() << "elements.";
	}
	
	compute_scalars(time);							// Get the scalars for the current time
	
	pos = (1 - this->s)*this->p1 + this->s*p2;
	vel = this->sd*(this->p2 - this->p1);
	acc = this->sdd*(this->p2 - this->p1);
}

/******************** Get the desired orientation for the given time ****************************************/
void Quintic::get_state(yarp::sig::Matrix &rot,					// Get desired state for orientation trajectory
		yarp::sig::Vector &vel,
		yarp::sig::Vector &acc,
		const float &time)
{
	if(rot.rows() != 3 || rot.cols() != 3)
	{
		yError() << "Quintic::get_state() : Expected an SO3 matrix, but you input a"
			<< rot.rows() << "x" << rot.cols() << "matrix!";
	}
	if(vel.size() != 3 || acc.size() != 3)
	{
		yError() << "Quintic::get_state() : This is an orientation trajectory and"
			<< "the input vectors should have a length of 3. Velocity vector has" << vel.size()
			<< "elements and acceleration has" << acc.size() << "elements.";
	}
	
	compute_scalars(time);							// Scalars for interpolating the trajectory.
	
	double temp = this->s*this->axisAngle[3];				// Interpolate along the angle
	
	yarp::sig::Vector blah({this->axisAngle[0],				// Interpolated orientation as axis-angle
				this->axisAngle[1],
				this->axisAngle[2],
				temp});
				
	rot = this->R1*yarp::math::axis2dcm(blah).submatrix(0,2,0,2);		// R(t) = R(0)*R(t)
	for(int i = 0; i < 3; i++)
	{
		vel[i] = this->sd*this->axisAngle[3]*this->axisAngle[i];	// Angular velocity
		acc[i] = this->sdd*this->axisAngle[3]*this->axisAngle[i];	// Angular acceleration
	}
}

/******************** Compute the coefficients for the polynomial ********************/
void Quintic::compute_coefficients()
{
	float dt = this->t2 - this->t1;						// Total time of trajectory
	this->a =   6*pow(dt,-5);
	this->b = -15*pow(dt,-4);
	this->c =  10*pow(dt,-3);
}

/******************** Compute scalars to interpolate along the trajectory ********************/
void Quintic::compute_scalars(const float &time)
{
	if(time < this->t1)							// Not yet started
	{
		this->s   = 0.0;
		this->sd  = 0.0;
		this->sdd = 0.0;
	}
	else if(time < this->t2)						// Somewhere inbetween
	{
		float dt = time - this->t1;					// Elapsed time since start
		this->s   =    this->a*pow(dt,5) +    this->b*pow(dt,4) +   this->c*pow(dt,3);
		this->sd  =  5*this->a*pow(dt,4) +  4*this->b*pow(dt,3) + 3*this->c*pow(dt,2);
		this->sdd = 20*this->a*pow(dt,3) + 12*this->b*pow(dt,2) + 6*this->c;
	}
	else									// Must be finished
	{
		this->s   = 1.0;
		this->sd  = 0.0;
		this->sdd = 0.0;
	}
}

#endif
