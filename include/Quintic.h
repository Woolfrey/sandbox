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
		// Constructor(s)
		Quintic() {}							// Empty constructor
		
		Quintic(const yarp::sig::Vector &startPoint,			// Trajectory between two points
			const yarp::sig::Vector &endPoint,
			const float &startTime,
			const float &endTime);
			
		Quintic(const yarp::sig::Matrix &startRotation,			// Trajectory between two orientations
			const yarp::sig::Matrix &endRotation,
			const float &startTime,
			const float &endTime);
		
		// Get Functions
		void get_state(yarp::sig::Vector &pos,				// Get the desired position
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &time);
		
		void get_state(yarp::sig::Matrix &rot,				// Get the desired rotation
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &time);
		
	private:
		// Variables
		bool isOrientation;						// Type of trajectory
		float t1, t2;							// Start and end of trajectory
		float a, b, c;							// Polynomial coefficients
		float s, sd, sdd;						// Scalars for interpolation
		yarp::sig::Matrix R1, R2;					// Used in orientation trajectories
		yarp::sig::Vector axisAngle;					// Used in orientation trajectories
		yarp::sig::Vector p1, p2;					// Start point and end point
	
		float angle;
		yarp::sig::Vector axis;
		
		// Functions
		void compute_coefficients();
		void compute_scalars(const float &time);
		
};										// Semicolon needed after class declaration

/******************** Constructor for position trajectory ****************************************/
Quintic::Quintic(const yarp::sig::Vector &startPoint,				// Trajectory between two points
		const yarp::sig::Vector &endPoint,
		const float &startTime,
		const float &endTime)
		:
		isOrientation(false),
		p1(startPoint),
		p2(endPoint),
		t1(startTime),
		t2(endTime)
{
	// Check the inputs are sound
	if(startPoint.size() != endPoint.size())
	{
		yError("Quintic::Quintic() : Inputs vectors are not of equal length!");
	}
	if(startTime > endTime)
	{
		yError("Quintic::Quintic() : Start time is greater than end time! Swapping the values...");
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
		isOrientation(true),
		R1(startRotation),
		R2(endRotation),
		t1(startTime),
		t2(endTime)
{
	// Check the inputs are sound
	if(startRotation.rows() != 3 || startRotation.cols() != 3 || endRotation.rows() != 3 || endRotation.cols() != 3)
	{
		yError("Quintic::Quintic() : Expected SO3 matrices for the input rotations!");
		yError() << "The size of the inputs are " << startRotation.rows() << "x" << startRotation.cols()
			 << " and " << endRotation.rows() << "x" << endRotation.cols() << " respectively.";
	}
	if(startTime > endTime)
	{
		yError("Quintic::Quintic() : Trajectory starts before it ends! Swapping the start and end times...");
		float temp = this->t2;
		this->t2 = this->t1;
		this->t1 = temp;
	}
	
	// We need to interpolate over the DIFFERENCE in orientation
	// R1*dR = R2 ----> dR = R1'*R2
	yarp::sig::Matrix dR = this->R1.transposed()*this->R2;				// Difference in rotation
	yarp::sig::Vector temp = yarp::math::dcm2axis(dR);				// Extract the axis-angle representation
	
	std::cout << "Here is the angle:" << temp[3]*180/M_PI << std::endl;
	if(temp[3] > M_PI)								// If the rotation is greater than 180 degrees...
	{	
		temp[3] = 2*M_PI - temp[3];						// ... Take the shorter route
		for(int i = 0; i < 3; i++) temp[i] *= -1;				// Flip the axis to match
	}
	
	this->axisAngle.resize(3);
	for(int i = 0; i < 3; i++) this->axisAngle[i] = temp[3]*temp[i];		// We will interpolate over angle*axis
	
	compute_coefficients();
}

/******************** Get the state for a position trajectory ****************************************/
void Quintic::get_state(yarp::sig::Vector &pos,
			yarp::sig::Vector &vel,
			yarp::sig::Vector &acc,
			const float &time)
{
	// Check the inputs are sound
	if(pos.size() != vel.size() || vel.size() != acc.size())
	{
		yError() << "Quintic::get_state() : Vectors are not of equal length!"
			<< "pos is length" << pos.size() << ", vel is length" << vel.size()
			<< ", and acc is length" << acc.size();
	}
	if(this->isOrientation)
	{
		yError("Quintic::get_state() : Called the function for position, but this is an orientation trajectory! How did that happen?");
	}
	
	// Determine where we are on the trajectory and return the desired state
	if(time < this->t1)								// Not yet started
	{
		pos = this->p1;
		vel.zero();
		acc.zero();
	}
	else if(time < this->t2)							// Somewhere in the middle
	{
		compute_scalars(time);
		pos = (1.0 - this->s)*this->p1 + this->s*this->p2;
		vel = this->sd*(this->p2 - this->p1);
		acc = this->sdd*(this->p2 - this->p1);
	}
	else										// Must be finished
	{
		pos = this->p2;
		vel.zero();
		acc.zero();
	}
}

/******************** Get the state for an orientation trajectory ****************************************/
void Quintic::get_state(yarp::sig::Matrix &rot,
			yarp::sig::Vector &vel,
			yarp::sig::Vector &acc,
			const float &time)
{
	// Check the inputs are sound
	if(rot.rows() != 3 || rot.cols() != 3)
	{
		yError() <<"Quintic::get_state() : Expected an SO3 for the rotation argument."
			<< "You input a" << rot.rows() << "x" << rot.cols() << "matrix.";
	}
	if(vel.size() != 3)
	{
		yError() << "Quintic::get_state() : Expected a vector of length 3 for the vel argument."
			<< "Your input had a length of" << vel.size();
	}
	if(acc.size() != 3)
	{
		yError() << "Quintic::get_state() : Expected a vector of length 3 for the acc argument."
			<< "Your input had a length of" << acc.size();
	}
	if(!this->isOrientation)
	{
		yError("Quintic::get_state() : Called the function for orientation, but this is a position trajectory. How did that happen?");
	}
	
	// Determine where we are on the trajectory
	if(time < this->t1)							// Not yet started
	{
		rot = this->R1;
		vel.zero();
		acc.zero();
	}
	else if(time < this->t2)						// Somewhere in the middle
	{
		compute_scalars(time);						// Compute the scalars for the given time
		
		// Interpolate over angle*axis, where starting angle is zero
		yarp::sig::Vector temp = this->s*this->axisAngle;		// Get the interpolated aixs*angle
		this->angle = yarp::math::norm(temp);				// Angle is just the norm of the vector
//	s	std::cout << "Interpolated angle:" << this->angle <<  std::endl;
		this->axis = temp/angle;					// Get the unit vector for the axis
		
		yarp::sig::Matrix blah = yarp::math::axis2dcm(yarp::sig::Vector({this->axis[0],
									this->axis[1],
									this->axis[2],
									this->angle}));
		rot = this->R1*blah.submatrix(0,2,0,2);				// YARP is annoying!
		vel = this->sd*this->axisAngle;
		acc = this->sdd*this->axisAngle;
	}
	else									// Must be at the end
	{
		rot = this->R2;
		vel.zero();
		acc.zero();
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

/*
class Quintic
{
	public:
		// Constructor(s)
		Quintic() {}							// Empty constructor
		
		Quintic(const yarp::sig::Vector &startPoint,			// Constructor for trajectory between 2 points
			const yarp::sig::Vector &endPoint,
			const float &_startTime,
			const float &_endTime);
			
			
		Quintic(const yarp::sig::Matrix &startRotation,
			const yarp::sig::Matrix &endRotation,
			const float &_startTime,
			const float &_endTime);
			
		Quintic(const yarp::sig::Vector &_axisAngle,			// Constructor for rotation about an axis
			const float &_t1,
			const float &_t2);

		void get_state(yarp::sig::Vector &pos,				// Get the desired state in the real numbers
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &time);
				
		/*void get_state(yarp::sig::Vector &_axisAngle,			// Get the desired rotation as axis-angle
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &time);*/
/*		
	private:
		int type = 100;							// 1 = Real numbers, 2 = Angle-Axis
		
		float a, b, c;							// Polynomial coefficients
		
		float s, sd, sdd;						// Interpolation coefficients
		
		float t1, t2;							// Start time and end time
		
		yarp::sig::Vector p1, p2;					// Positions in the real numbers
		
		yarp::sig::Vector axisAngle;					// Axis-Angle representation for orientation
		
		void compute_coefficients();					// Compute the polynomial coefficients
		void compute_scalars(const float &time);			// Compute scalars for interpolation
		
};										// Need a semicolon after a class declaration

/******************** Constructor for a trajectory over the real numbers ********************
Quintic::Quintic(const yarp::sig::Vector &_p1,
		const yarp::sig::Vector &_p2,
		const float &_t1,
		const float &_t2)
		:
		p1(_p1),
		p2(_p2),
		t1(_t1),
		t2(_t2)
{
	// Check that input dimensions are correct
	if(this->p1.size() != this->p2.size())
	{
		yError()<< "Quintic::Quintic(): Input vectors are not of equal size."
			<< "The size of _p1 is" << p1.size() << "and the size of _p2 is" << p2.size() << ".";
				
	}
	if(this->t1 >= this->t2)
	{
		yError("Quintic::Quintic(): Trajectory ends before it begins! Swapping the start time and end time...");
		float temp = this->t1;
		this->t1 = this->t2;						// Swap the vectors so the times are correct
		this->t2 = temp;
	}
	
	this->type = 1;								// Tell the object to interpolate over reals in the future
	
	compute_coefficients();							// Compute polynomial coefficients
}

/******************** Constructor for an angle about a single axis ********************
Quintic::Quintic(const yarp::sig::Vector &_axisAngle,
		const float &_t1,
		const float &_t2)
		:
		axisAngle(_axisAngle),
		t1(_t1),
		t2(_t2)
{
	// Check input dimensions are sound
	if(axisAngle.size() != 4) yError("Quintic::Quintic(): axisAngle vector must have 4 elements.");
	if(this->t1 >= this->t2)
	{
		yError("Quintic::Quintic(): Trajectory ends before it begins! Swapping the start time and end time...");
		float temp = this->t1;
		this->t1 = this->t2;						// Swap the vectors so the times are correct
		this->t2 = temp;
	}
	
	if(this->axisAngle[3] > 3.1416)						// Rotation is greater than 180 degrees
	{
		this->axisAngle[3] = 2*3.1416 - this->axisAngle[3];		// Take the short path
		for(int i = 0; i < 3; i++) this->axisAngle[i] *= -1;		// Flip the axis to match
	}
	
	axisAngle.subVector(0,2) = axisAngle.subVector(0,2)/yarp::math::norm(axisAngle.subVector(0,2)); // Ensure unit norm for good measure
	
	this->type = 2 ;							// Tell the object to interpolate over axis-angle in the future
		
	compute_coefficients();							// Compute polynomial coefficients
}

/******************** Get the desired state ********************
void Quintic::get_state(yarp::sig::Vector &pos,
			yarp::sig::Vector &vel,
			yarp::sig::Vector &acc,
			const float &time)
{
	//compute_scalars(time);						// Get the interpolation scalars for the given time
	
	if(time < this->t1)							// Not yet started, remain at the start
	{
		this->s   = 0.0;
		this->sd  = 0.0;
		this->sdd = 0.0;
	}
	else if(time <= this->t2)						// Somewhere inbetween
	{
		float dt = time - this->t1;					// Time since start
		
		this->s   =    this->a*pow(dt,5) +    this->b*pow(dt,4) +   this->c*pow(dt,3);
		this->sd  =  5*this->a*pow(dt,4) +  4*this->b*pow(dt,3) + 3*this->c*pow(dt,2);
		this->sdd = 20*this->a*pow(dt,3) + 12*this->b*pow(dt,2) + 6*this->c;
	}
	else									// Finished, remain at the end
	{
		this->s   = 1.0;
		this->sd  = 0.0;
		this->sdd = 0.0;
	}
	
	// Interpolate based on type of trajectory	
	switch(this->type)
	{
		case 1:							// Interpolation over the real numbers
		
			if(pos.size() != vel.size() || vel.size() != acc.size())
			{
				yError() << "Quintic::get_state(): Input vectors are not of equal length."
					<< "The size of pos is" << pos.size() << ","
					<< "the size of vel is" << vel.size() << ","
					<< "and the size of acc is" << acc.size() << ".";
			}
			
				
			for(int i = 0; i < pos.size(); i++)
			{
				pos[i] = (1 - this->s)*this->p1[i] + this->s*this->p2[i];
				vel[i] = this->sd*(this->p2[i] - this->p1[i]);
				acc[i] = this->sdd*(this->p2[i] - this->p1[i]);
			}
			break;
		case 2:							// Interpolation over axis-angle
			
			if(pos.size() != 4)
			{
				yError() << "Quintic::get_state(): Position vector must have 4 elements";
			}
			else if(vel.size()!= 3 || acc.size() != 3)
			{
				yError() << "Quintic::get_state(): vel and acc vectors must have 3 elements."
					<< "The size of vel is" << vel.size()
					<< "and the size of acc is" << acc.size() << ".";
			}
			
			pos[3] = this->s*this->axisAngle[3];			// Interpolate the angle (remember, it starts from 0)
			
			for(int i = 0; i < 3; i++)
			{
				pos[i] = this->axisAngle[i];					// Unchanged, but convenient to return it here.
				vel[i] = this->sd*this->axisAngle[3]*this->axisAngle[i];	// theta_dot*axis
				acc[i] = this->sdd*this->axisAngle[3]*this->axisAngle[i];	// theta_ddot*axis
			}
			
			break;
		default:
			yError("Quintic::get_state(): Type incorrectly specified.");
			break;
	}
}

/******************** Compute the coefficients for the polynomial ********************
void Quintic::compute_coefficients()
{
	float dt = this->t2 - this->t1;					// Total time of trajectory
	this->a =   6*pow(dt,-5);
	this->b = -15*pow(dt,-4);
	this->c =  10*pow(dt,-3);
}
#endif*/
