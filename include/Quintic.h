#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>

class Quintic
{
	public:
		Quintic();							// Empty constructor
		
		Quintic(const yarp::sig::Vector &_p1,				// Constructor for points in the real numbers
			const yarp::sig::Vector &_p2,
			const float &_t1,
			const float &_t2);
			
		Quintic(const yarp::sig::Vector &_axisAngle,			// Constructor for rotation about an axis
			const float &_t1,
			const float &_t2);

		void get_state(yarp::sig::Vector &pos,			// Get the desired state in the real numbers
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &time);
				
		/*void get_state(yarp::sig::Vector &_axisAngle,			// Get the desired rotation as axis-angle
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &time);*/
		
	private:
		int type = 100;						// 1 = Real numbers, 2 = Angle-Axis
		
		float a, b, c;							// Polynomial coefficients
		
		float s, sd, sdd;						// Interpolation coefficients
		
		float t1, t2;							// Start time and end time
		
		yarp::sig::Vector p1, p2;					// Positions in the real numbers
		
		yarp::sig::Vector axisAngle;					// Axis-Angle representation for orientation
		
		void compute_coefficients();					// Compute the polynomial coefficients
		void compute_scalars(const float &time);			// Compute scalars for interpolation
		
};										// Need a semicolon after a class declaration

/******************** Empty constructor ********************/
Quintic::Quintic()
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

/******************** Constructor for a trajectory over the real numbers ********************/
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
	
	this->type = 1;							// Tell the object to interpolate over reals in the future
	
	compute_coefficients();						// Compute polynomial coefficients
}

/******************** Constructor for an angle about a single axis ********************/
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
	
	if(this->axisAngle[3] > M_PI)						// Rotation is greater than 180 degrees
	{
		this->axisAngle[3] = 2*M_PI - this->axisAngle[3];		// Take the short path
		for(int i = 0; i < 3; i++) this->axisAngle[i] *= -1;		// Flip the axis to match
	}
	
	axisAngle.subVector(0,2) = axisAngle.subVector(0,2)/yarp::math::norm(axisAngle.subVector(0,2)); // Ensure unit norm for good measure
	
	this->type = 2 ;							// Tell the object to interpolate over axis-angle in the future
		
	compute_coefficients();						// Compute polynomial coefficients
}

/******************** Get the desired state ********************/
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

/******************** Compute the coefficients for the polynomial ********************/
void Quintic::compute_coefficients()
{
	float dt = this->t2 - this->t1;					// Total time of trajectory
	this->a =   6*pow(dt,-5);
	this->b = -15*pow(dt,-4);
	this->c =  10*pow(dt,-3);
}


/******************** Get the desired rotation ********************
void Quintic::get_state(yarp::sig::Vector &_axisAngle,
			yarp::sig::Vector &vel,
			yarp::sig::Vector &acc,
			const float &time)
{
	compute_scalars(time);							// Get the interpolation scalars for the given time
	
	_axisAngle[3] = this->s*this->axisAngle[3];				// Interpolate the angle
	
	for(int i = 0; i < 3; i++)
	{
		_axisAngle[i]	= this->axisAngle[i];				// Axis remains unchanged, but convenient to return it here
		vel[i]       	= this->sd*this->axis[i];			// Angular velocity
		acc[i]       	= this->sdd*this->axis[i];			// Angular acceleration
	}
}


/******************** Compute interpolation coefficients at the given time ********************
void Quintic::compute_scalars(const float &time)
{
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
}*/
