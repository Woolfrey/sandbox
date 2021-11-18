#include <CartesianTrajectory.h>							// Custom trajectory class
#include <iCub/iKin/iKinFwd.h>							// iCub::iKin::iCubArm object
#include <MultiJointController.h>							// This handles the low-level joint interface

class ArmController :	public iCub::iKin::iCubArm,
			public MultiJointController,
			virtual public yarp::os::PeriodicThread
{
	public:
		ArmController() : yarp::os::PeriodicThread(0.01){};
		
		void configure(const std::string &local_port_name,			// Configure the device drivers
				const std::string &remote_port_name,
				const std::string &_name,
				const int &number_of_joints);
		
		void move_to_pose(const yarp::sig::Matrix &desiredPose, const float &time); // Move the hand to a desired pose
		
		/******************************************************************/
		virtual bool initThread();						// Executed just after start() is called
		virtual void run();							// Executed just after initThread()
		virtual void releaseThread();						// Executed just after stop() is called
		/******************************************************************/
				
	private:
		float startTime, elapsedTime;						// Used for timing in the control loop
		
		CartesianTrajectory trajectory;					// Trajectory object used in control
		
		yarp::sig::Matrix pos;							// Desired pose as a 4x4 matrix
		yarp::sig::Vector vel, acc;						// Desired velocity, acceleration as 6x1 vectors
		
		yarp::sig::Vector get_pose_error(const yarp::sig::Matrix &desired,
						  const yarp::sig::Matrix &actual);
						  
		yarp::sig::Matrix get_weighted_inverse(const yarp::sig::Matrix &J,
							const yarp::sig::Matrix &W);
		
		yarp::sig::Matrix get_joint_weighting();
		
		
};											// Semicolon needed after class declaration


/******************** Move the hand to a desired pose ********************/
void ArmController::move_to_pose(const yarp::sig::Matrix &desiredPose, const float &endTime)
{
		if(desiredPose.rows() != 4 || desiredPose.cols() !=4)		// Ensure input dimensions are sound before proceeding
		{
			yError("ArmController::move_to_pose() : Expected a 4x4 homogeneous transform");
		}
		else
		{
			if(isRunning()) PeriodicThread::stop();			// Stop any Cartesian control threads
			if(MultiJointController::isRunning()) MultiJointController::stop(); // Stop any joint control threads
			
			update_state();						// Update joint state
			setAng(this->q*M_PI/180);					// Set joint angles in iCubArm class
			
			yarp::sig::Matrix currentPose = getH();			// Get the current end-effector pose
			
			/***** NOTE TO SELF: Check that end_time is reasonable *****/
			
			this->trajectory = CartesianTrajectory(currentPose, desiredPose, 0.0, endTime);
			
			start();							// Go to initThread()
		}
}

/******************** Executed just before run() is called ********************/
bool ArmController::initThread()
{
	this->startTime = yarp::os::Time::now();					// Record the start time for the control loop
	return true;
	// Now immediately go to run() function
}

/******************** Main control loop ********************/
void ArmController::run()
{
	update_state();								// Get the current joint state from the robot
	setAng(this->q*M_PI/180);							// Set the angles (convert from deg to rad)
	
	this->elapsedTime = yarp::os::Time::now() - this->startTime;			// Elapsed time in control loop
	
	this->trajectory.get_state(this->pos, this->vel, this->acc, this->elapsedTime); // Get new desired state
	
	yarp::sig::Vector xdot = vel + 2.0*get_pose_error(pos, getH());		// Feedforward + feedback
	
	// Construct the redundant task to stay within joint limits
	yarp::sig::Vector redundant(this->n);
	double qMin, qMax;
	for(int i = 0; i < this->n; i++)
	{
		this->limits->getLimits(i, &qMin, &qMax);
		redundant[i] = 0.5*(qMax + qMin) - this->q[i];
	}
	
	// Kinematic stuff
	yarp::sig::Matrix J = GeoJacobian();						// Get the Jacobian matrix at current state
	yarp::sig::Matrix invJ = get_weighted_inverse(J, get_joint_weighting());	// Pseudoinverse Jacobian
	yarp::sig::Matrix I(this->n, this->n);					// Identity matrix
	I.eye();
	
	move_at_speed(invJ*xdot + (I - invJ*J)*redundant);				// RMRC with null space projection
}

/******************** Executed immediately after stop() is called ********************/
void ArmController::releaseThread()
{
	for(int i = 0; i < this->n; i++)
	{
		this->controller->velocityMove(i, 0.0);				// Stop any joints moving
	}
}


/******************** Get the error between two poses ********************/
yarp::sig::Vector ArmController::get_pose_error(const yarp::sig::Matrix &desired, const yarp::sig::Matrix &actual)
{
	/*** NOTE TO SELF: If there is a problem with orientation feedback, it's probably here ***/
	
	yarp::sig::Vector axisAngle = yarp::math::dcm2axis(desired*yarp::math::SE3inv(actual)); // Get the rotation error
	
	if(axisAngle[0] > 3.1416)							// If angle > 180 degrees...
	{
		axisAngle[0] = 2*3.1416 - axisAngle[0];				// ... Take the shorter path...
		for(int i = 1; i < 4; i++) axisAngle[i] *= -1;			// ... And flip the axis to match
	}
	
	
	yarp::sig::Vector error(6);							// Value to be returned
	
	for(int i = 0; i < 3; i++)
	{
		error[i]	= desired[i][3] - actual[i][3];			// Position error
		error[i+3]	= axisAngle[0]*axisAngle[i+1];			// Orientation error
	}
	
	return error;
}

/******************** Get the weighted pseudoinverse - assumes W is inverted ********************/
yarp::sig::Matrix ArmController::get_weighted_inverse(const yarp::sig::Matrix &J, const yarp::sig::Matrix &W)
{
	yarp::sig::Matrix WJt = W*J.transposed();					// This makes things a little easier
	
	yarp::sig::Matrix A = J*WJt;							// We need to invert this
	
	// Perform SVD
	yarp::sig::Matrix U(6,6), V(6,6);
	yarp::sig::Vector s(6);
	yarp::math::SVD(A,U,s,V);							// Singular value decomposition
	
	yarp::sig::Matrix invA(this->n, 6);						// Value to be returned
	invA.zero();									// Set as all zero

	// Perform first part of inversion
	for(int i = 0; i < this->n; i++)
	{
		for(int j = 0; j < 6; j++)
		{
			if(s(j) > 1e-03)	invA(i,j) = V(i,j)/s(j);		// This skips off-diagonals in S matrix
			else if(s(j) > 0)	invA(i,j) = V(i,j)*1e03;		// Cap the values to avoid singularities
		}
	}
	
	return WJt*invA*U.transposed();						// Perform second half of inversion and return
}

/******************** Get weighting to avoid joint limits ********************/
yarp::sig::Matrix ArmController::get_joint_weighting()
{
	yarp::sig::Matrix W(this->n, this->n);					// Value to be returned
	W.eye();									// Set as identity
	
	double qMin, qMax, upper, lower, range, dpdq;
	for(int i = 0; i < this->n; i++)
	{
		this->limits->getLimits(i, &qMin, &qMax);				// Get joint limits for the ith joint
		upper = qMax - this->q[i];						// Distance to the upper limit
		lower = this->q[i] - qMin;						// Distance from the lower limit
		range = qMax - qMin;							// Range between limits
		dpdq = 1/pow(upper,2) - 1/pow(lower, 2);				// Gradient
		
		if(dpdq*this->qdot[i] > 0)						// Moving toward a joint limit
		{
			W[i][i] /= (range/(upper*lower) - 4/range + 1);		// This is actually the INVERSE of the penalty
		}
	}
	
	return W; // Note: this is the INVERSE
}

/******************** Configure the arm and device drivers ********************/
void ArmController::configure(const std::string &local_port_name,
				const std::string &remote_port_name,
				const std::string &_name,
				const int &number_of_joints)
{
	allocate(_name+"2");								// Sets the iCubArm object
	
	configure_drivers(local_port_name,						// Establish connection with yarp ports
			remote_port_name,
			_name+" arm",
			number_of_joints);
}


/*
class ArmController : 	public iCub::iKin::iCubArm,
			public MultiJointController
{
	public:
		ArmController();							// Constructor
		
		void configure(const std::string &local_port_name,
				const std::string &remote_port_name,
				const std::string &_name,
				const int &number_of_joints);				// Configure the device drivers
		
		void rmrc(const yarp::sig::Vector &speed);				// Resolved motion rate control (RMRC)
		
		void rmrc(const yarp::sig::Vector &speed,
			  const yarp::sig::Vector &secondaryTask);			// RMRC with null space projection

		double period;
		
		yarp::sig::Matrix get_joint_weighting();				// Weighting matrix for joint limit avoidance
			
		yarp::sig::Matrix get_pseudoinverse(const yarp::sig::Matrix &J);	// Get the pseudoinverse of the Jacobian
	
};											// Semicolon needed after class declaration

/******************** Constructor ********************
ArmController::ArmController()
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

/******************** Move the endpoint at a given speed********************
void ArmController::rmrc(const yarp::sig::Vector &speed)
{
	yarp::sig::Vector temp(7);							// Empty null space vector
	temp.zero();									// All zeros
	rmrc(speed, temp);								// RMRC with null space projection
}
void ArmController::rmrc(const yarp::sig::Vector &speed, const yarp::sig::Vector &secondaryTask)
{
	yarp::sig::Vector control(7);
	control.zero();
	
	if(speed.size() !=6)
	{
		yError("ArmController::rmrc() : Expected a 6x1 vector for the input.");
	}
	else
	{
		yarp::sig::Matrix J = GeoJacobian(this->q);				// Get the Jacobian for the current joint angles
		
		yarp::sig::Matrix invJ = get_pseudoinverse(J);			// Get the weighted pseudoinverse Jacobian
		
		yarp::sig::Matrix I(7,7);						// Identity matrix
		I.eye();
		
		control = invJ*speed + (I - invJ*J)*secondaryTask;			// RMRC with null space projection
	}
	
	move_at_speed(control);
}

/******************** Get the pseudoinverse of the Jacobian ********************
yarp::sig::Matrix ArmController::get_pseudoinverse(const yarp::sig::Matrix &J)
{
	yarp::sig::Matrix W = get_joint_weighting();					// NOTE: This is actually the inverse already!
	yarp::sig::Matrix A = J*W*J.transposed();					// This matrix needs to be inverted
	yarp::sig::Matrix U(6,6), V(6,6);						// Orthogonal matrices
	yarp::sig::Vector s(6);							// Vector of singular values
	yarp::math::SVD(A, U, s, V);							// Do the singular value decomposition
	yarp::sig::Matrix invA(6,6);							// Inverse to be computed
	
	// Compute the partial inverse
	for(int i = 0; i < 6; i++)
	{
		for(int j = 0; j < 6; j++)
		{
			if(s(j) > 1e-03)	invA(i,j) = V(i,j)/s(j);		// Proper inverse
			else			invA(i,j) = V(i,j)*1e03;		// Damp near-singular directions	
		}
	}
	
	return W*J.transposed()*invA*U.transposed();					// This is the full pseudoinverse
}

/******************** Get the weighting matrix for joint limit avoidance ********************
yarp::sig::Matrix ArmController::get_joint_weighting()
{
	// NOTE: This function currently returns the INVERSE of the matrix, to save
	// on computational cost. Might need to change this in the future when the inertia
	// matrix is added.
	
	// Reference:
	// Chan, T. F., & Dubey, R. V. (1995). A weighted least-norm solution based
	// scheme for avoiding joint limits for redundant joint manipulators.
	// IEEE Transactions on Robotics and Automation, 11(2), 286-292.
	
	yarp::sig::Matrix W(this->n,this->n);						// Value to be returned
	W.eye();									// Set as identity
	double qMin, qMax;								// Store min. and max. values here
	float dwdq;									// Partial derivative
	float dMin, dMax;								// Distance from upper and lower limits
	for(int i = 0; i < 7; i++)
	{
		this->limits->getLimits(i, &qMin, &qMax);				// Get the upper and lower bounds for this joint
		dMin = this->q[i] - qMin;						// Distance from the lower limit
		dMax = qMax - this->q[i];						// Distance from the upper limit
		dwdq = 1/pow(dMax,2) - 1/pow(dMin,2);					// Partial derivative of the penalty function	
		
		// Override the identity if the joint is moving towards a limit
		if(dwdq*this->qdot[i] > 0) W[i][i] = 1.0/((qMax - qMin)/(dMax*dMin) - 4/(qMax-qMin) + 1);
	}
	
	return W; // NOTE: Currently the inverse
}

/******************** Configure the arm and device drivers ********************
void ArmController::configure(const std::string &local_port_name,
				const std::string &remote_port_name,
				const std::string &_name,
				const int &number_of_joints)
{
	configure_drivers(local_port_name, remote_port_name, _name, number_of_joints);
}

*/
