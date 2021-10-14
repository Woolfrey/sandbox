#include <yarp/math/Math.h>								// Math functions for yarp
#include <yarp/sig/Matrix.h>								// yarp::sig::Matrix
#include <yarp/sig/Vector.h>								// yarp::sig::Vector
#include <vector>									// std::vector


class CubicSpline
{
	public:
		CubicSpline();								// Empty constructor
		
		CubicSpline(const std::vector<yarp::sig::Vector> &_points,		// Proper constructor
			    const std::vector<float> &_times);
			    
		void get_state(yarp::sig::Vector &pos,				// Get the state for the current time
				yarp::sig::Vector &vel,
				yarp::sig::Vector &acc,
				const float &t);
	private:
		int m, n;								// m dimensions, n waypoints (n-1 splines)
		
		std::vector<float> time;						// Time to pass each waypoints
		
		std::vector<yarp::sig::Vector> a, b, c, d;				// Spline coefficients
		

};											// Semicolon needed after a class declaration
/******************** Empty constructor ********************/
CubicSpline::CubicSpline()
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}
/******************** Basic constructor ********************/
CubicSpline::CubicSpline(const std::vector<yarp::sig::Vector> &_points,
			const std::vector<float> &_times)
		:
		m(_points[0].size()),							// Get the number of dimensions
		n(_points.size()),							// Get the number of waypoints
		time(_times)								// Transfer the times for each waypoint
{
	// Check inputs are sound
	for(int i = 0; i < this->n-1; i++)
	{
		if(_points[i].size() != _points[i+1].size())
		{
			yError("CubicSpline::CubicSpline() : Input vectors are not of equal length.");
		}
		if(_times[i] >= _times[i+1])
		{
			yError("CubicSpline::CubicSpline() : Input times are not in ascending order.");
			// Sort them here?
		}
	}
	
	// Resize arrays, vectors accordingly
	for(int i = 0; i < this->n-1; i++)						// Remember, there are only n-1 splines!
	{
		this->a.push_back(yarp::sig::Vector(this->m));
		this->b.push_back(yarp::sig::Vector(this->m));
		this->c.push_back(yarp::sig::Vector(this->m));
		this->c.push_back(yarp::sig::Vector(this->m));
	}
	
	// We know the waypoints, s(i) for i = 1, ..., n.
	// We want to determine the accelerations sddot(i).
	// They are related by A*sddot = B*s
	
	yarp::sig::Matrix A(this->n, this->n);
	yarp::sig::Matrix B(this->n, this->n);
	A.zero();
	B.zero();
	
	// Compute the values for the first entries in the matrices
	float dt1 = this->time[1] - this->time[0];
	float dt2 = this->time[2] - this->time[1];
	A(0,0) = dt1/2;
	A(0,1) = dt1/2 + dt2/3;
	A(0,2) = dt2/6;
	// SPECIAL RULE FOR ANGLE-AXIS HERE
	B(0,1) = 1/dt2;
	
	// Compute the values for the middle entries in the matrices
	for(int i = 1; i < this->n-1; i++)
	{
		dt1 = this->time[i] - this->time[i-1];
		dt2 = this->time[i+1] - this->time[i];
		
		A[i][i-1] = dt1/6.0;
		A[i][i]  = (dt1 + dt2)/3.0;
		A[i][i+1] = dt2/6.0;
		
		// SPECIAL RULE FOR QUATERNION HERE???
		B(i,i-1) = -1/dt1;
		B(i,i) = 1/dt2;
	}
	
	// Compute the values for the final values in the matrices
	float dt = this->time[this->n-1] - this->time[this->n-2];
	A(this->n-1, this->n-2) = dt/6;
	A(this->n-1, this->n-1) = dt/3;
	// SPECIAL RULE FOR ANGLE-AXIS HERE
	B(this->n-1, this->n-2) = -1/dt;
	
	yarp::sig::Matrix C = yarp::math::pinv(A);//*B;				// This makes future calcs a little easier
	
	// Compute coefficients for all the splines across each dimension
	yarp::sig::Vector s(this->n), sdd(this->n);					// Vectors of position, acceleration
	
	for(int i = 0; i < this->m; i++)
	{
		for(int j = 0; j < this->n; j++) s[j]	= _points[i][j];		// Get all the points along the ith dimension
		
		// Perform the product sdd = C*j
		sdd.zero();
		for(int j = 0; j < this->n; j++)
		{
			for(int k = 0; k < this->n; j++) sdd[j] += C[j][k]*s[k];
		}	
		
		// Compute the coefficients for the n-1 splines
		for(int j = 0; j < this->n-1; j++)
		{
			float dt = this->time[j+1] - this->time[j];			// Time difference between waypoints
			
			this->a[i][j] = (sdd[j+1] - sdd[j])/(6*dt);
			this->b[i][j] = sdd[j]/2;
			this->c[i][j] = (s[j+1] - s[j])/dt - dt*(sdd[j+1] + 2*sdd[j])/6;
			this->d[i][j] = s[j];
		}
		
		// SPECIAL RULE FOR ANGLE-AXIS HERE
	}
}

/******************** Get the current state ********************/
void CubicSpline::get_state(yarp::sig::Vector &pos,
			     yarp::sig::Vector &vel,
			     yarp::sig::Vector &acc,
			     const float &t)
{
	// First, figure out which spline we are on
	
	// Then, compute the state
		// Start: Position = start position, vel = 0, acc = 0
		
		// Middle: Do math
		
		// End: position = end position, vel = 0, acc = 0
	
}
