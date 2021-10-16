/*
*	IMPORTANT NOTES!
*
*		- Angles are in degrees, degrees/second - Need to convert them!
*		- There is a problem moving from the "shake" configuraiton to the
*		  "receive" configuration which is probably due to the logic in
*		  the optimal time scaling.
*
*/

#include <iostream>									// std::cout << std::endl;
#include <iCub/iKin/iKinFwd.h>							// iCub::iKin::iCubArm object
#include <yarp/dev/ControlBoardInterfaces.h>						// Communicates with motor controllers on the robot?
#include <yarp/dev/PolyDriver.h>							// Device driver class
#include <yarp/math/SVD.h>								// yarp::math::pinv() and the like
#include <yarp/os/LogStream.h>							// yarp::Info() and the like
#include <yarp/os/Property.h>								// Don't know exactly what this does
#include <CartesianTrajectory.h>							// Custom trajectory class

class ArmController : public iCub::iKin::iKinLimb
{
	public:
		ArmController(	const std::string &local_port_name,
				const std::string &remote_port_name,
				const std::string &_name);
				
		void close() {this->driver.close();}					// Closes the device drivers
		void move_to_position(yarp::sig::Vector &target);			// Move joints to a desired state
		void move_to_pose(yarp::sig::Matrix &target);				// Move hand to a desired state
		void move_hand();							// Move hand at a constant speed
		void move_up();							// Move the hand up 10cm
		void move_down();							// Move the hand down 10cm
		void move_left();							// Move the hand left 10cm
		void move_right();							// Move the hand right 10cm
		
	private:
		std::string name;							// Name of this object
		
		// Kinematics
		iCub::iKin::iCubArm arm;						// Forward kinematics & Jacobian
		int n = 7;								// 7 joints on the iCub arm
		yarp::sig::Vector q, qdot;						// Joint positions, velocities
		
		// These interface with the hardware
    		yarp::dev::IControlLimits* limits;					// Joint limits?
		yarp::dev::IControlMode* mode;					// Sets the control mode of the motor
		yarp::dev::IEncoders* encoder;					// Joint position values (in degrees)
		yarp::dev::IVelocityControl* controller;				// Motor-level velocity controller
		yarp::dev::PolyDriver driver;						// Device driver
		
		// Functions
		bool update_state();							// Updates the kinematics internally
		
		yarp::sig::Vector get_pose_error(const yarp::sig::Matrix &desired,	// Converts the pose error from SE(3)to a 6x1 vector for feedback control
						  const yarp::sig::Matrix &actual);				  
		yarp::sig::Matrix get_joint_weighting();				// Get the weighting matrix for joint limit avoidance
		yarp::sig::Matrix dls(const yarp::sig::Matrix &J);			// Get the DLS inverse of the Jacobian
		
		void rmrc(const yarp::sig::Vector &speed);
		
		
		int direction = 1;
		
		
};											// Semicolon needed after class declaration

/******************** Constructor ********************/
ArmController::ArmController(const std::string &local_port_name,
				const std::string &remote_port_name,
				const std::string &_name)
				:
				name(_name),
				arm(iCub::iKin::iCubArm(_name))
{
	this->q.resize(16);								// Joints 1-7 = Arm, 8:16 = Hand
	this->qdot.resize(7);								// Only controlling 7 joints
	
	// Configure the device driver
	yarp::os::Property options;							// Object to temporarily store options
	options.put("device", "remote_controlboard");				
	options.put("local", local_port_name);					// Local port names
	options.put("remote", remote_port_name);					// Where we want to connect to

	this->driver.open(options);							// Assign the options to the driver
	if(!this->driver.isValid())
	{
		yError("Unable to configure device. Here are the known devices:");
		yInfo() <<  yarp::dev::Drivers::factory().toString().c_str();	
	}		
	else				yInfo() << "Successfully configured device driver for" << this->name+" arm";
	
	// Then configure the joint controllers
	if(!this->driver.view(this->controller))
	{
		yError() << "Unable to configure the controller for" << this->name+" arm.";
		this->driver.close();
	}
	else if(!this->driver.view(this->mode)) yError() << "Unable to configure the control mode for" << this->name+" arm.";
	else
	{		
		for(int i = 0; i < this->n; i++)
		{
			this->mode->setControlMode(i, VOCAB_CM_VELOCITY);				// Set the motor in velocity mode
			this->controller->setRefAcceleration(i,std::numeric_limits<double>::max());	// CHANGE THIS?
			this->controller->velocityMove(i,0.0);					// Ensure initial velocity is zero
		}
		yInfo() << "Successfully configured velocity control of" << this->name+" arm.";
	}
	
	// Try and obtain the joint limits
	if(!this->driver.view(this->limits)) 	yError() << "Unable to obtain the joint limits for" << this->name+" arm.";
	else					yInfo() << "Successfully obtained limits for" << this->name+" arm.";

	// Finally, configure the encoders
	if(this->driver.view(this->encoder))
	{		
		for(int i = 0; i < 5; i++)
		{
			if(this->encoder->getEncoders(this->q.data()))		// Get initial encoder values
			{
				yInfo() << "Successfully configured the encoder for" << this->name+" arm.";
				break;							// This should exit the for-loop
			}
			if(i == 5) yError() << "Could not obtain encoder values for" << this->name+" arm" << "in 5 attempts.";		
			yarp::os::Time::delay(0.02);					// Wait a little bit and try again
		}
	}
	else yError() << "Could not configure the encoder for" << this->name+" arm.";
}

/******************** Move joints to a desired configuration ********************/
void ArmController::move_to_position(yarp::sig::Vector &target)
{		
	update_state();								// Get new state information
	
	// Check to see if all target positions are inside the joint limits
	double qMin, qMax;
	for(int i = 0; i < 7; i++)
	{
		this->limits->getLimits(i, &qMin, &qMax);
		if(target[i] > qMax) 		target[i] = qMax - 0.05;
		else if(target[i] < qMin)	target[i] = qMin + 0.05;
	}
	
	// Figure out optimal time scaling for the trajectory
	double vMin, vMax;
	float dt, dq;
	float end_time = 0.0;
	for(int i = 0; i < 7; i++)
	{
		dq = target[i] - this->q[i];						// Distance to target
		this->limits->getVelLimits(i, &vMin, &vMax);				// Get velocity limits for ith joint
		if(dq >= 0)	dt = (15*dq)/(8*vMax);					// Time to cover distance at max speed			
		else		dt = (15*dq)/(8*vMin);					
		if(dt > end_time) end_time = dt;					// Update based on maximum time
	}
	end_time *= 1.5;								// Make the time a bit longer

	Quintic trajectory(this->q.subVector(0,6), target, 0, end_time);		// Create trajectory object
	
	// Run the control
	bool finished = false;
	double elapsed_time;								// Time since start
	double start_time = yarp::os::Time::now();					// Start time for the trajectory
	yarp::sig::Vector q_d(this->n),						// Desired state values
			  qdot_d(this->n),
			  qddot_d(this->n),
			  control(this->n);						// Command speed (deg/s)
	while(!finished)
	{
		elapsed_time = yarp::os::Time::now() - start_time;			// Get current time
		
		update_state();							// Update kinematics
		
		if(elapsed_time <= end_time)
		{
			trajectory.get_state(q_d, qdot_d, qddot_d, elapsed_time);	// Current
			
			for(int i = 0; i < this->n; i++)
			{
				control[i] = qdot_d[i]					// Feedforward term
					   + 10.0*(q_d[i] - this->q[i]); 		// Feedback term
			}
		}
		else
		{
			control.zero();						// Stop
			finished = true;
		}		
		for(int i = 0; i < this->n; i++) this->controller->velocityMove(i, control[i]);
		yarp::os::Time::delay(0.005);						// Wait a bit
	}
}

/******************** Move hand at a given speed ********************/
void ArmController::rmrc(const yarp::sig::Vector &speed)
{
	if(speed.size() !=6)
	{
		yError("ArmController::rmrc() : Expected a 6x1 vector for the input.");
		
		for(int i = 0; i < this->n; i++)
		{
			this->controller->velocityMove(i, 0.0);			// Stop the arm moving
		}
	}
	else
	{
		yarp::sig::Matrix J = this->arm.GeoJacobian();			// Jacobian at current state (update before calling this function!)
		yarp::sig::Matrix invJ = dls(J);					// DLS inverse with joint weighting
		
		yarp::sig::Vector control(this->n);
		control.zero();
		for(int i = 0; i < this->n; i++)
		{
			for(int j = 0; j < 6; j++) control[i] += invJ(i,j)*speed[j];
		}
		
		for(int i = 0; i < this->n; i++) this->controller->velocityMove(i,control[i]*180/M_PI); // Convert to deg/s
	}
}

/******************** Move the hand to a desired pose ********************/
void ArmController::move_to_pose(yarp::sig::Matrix &target)
{
	update_state();								// Update current joint state
	yarp::sig::Matrix Ta = this->arm.getH();					// Get pose as homogeneous transformation matrix
	
	// Figure out optimal time scaling
	float end_time = 2.0;
	
	CartesianTrajectory trajectory(Ta, target, 0.0, end_time);			// Create trajectory object
	
	// Run the control
	bool finished = false;
	double elapsed_time;
	double start_time = yarp::os::Time::now();					// Get current time
	yarp::sig::Matrix Td(4,4);							// Desired pose as SE(3)
	yarp::sig::Vector xdot_d(6), xddot_d(6);					// Desired velocity, acceleration
	yarp::sig::Vector xe(6);							// Pose error
	yarp::sig::Vector xdot(6);
	yarp::sig::Matrix J(6,this->n);						// Jacobian matrix
	
	while(!finished)
	{
		elapsed_time = yarp::os::Time::now() - start_time;			// Elapsed time in control loop

		trajectory.get_state(Td, xdot_d, xddot_d, elapsed_time);		// Get desired state
		update_state();							// Update current joint states
		Ta = this->arm.getH();							// Get the current hand pose
		xe = get_pose_error(Td, Ta);						// Compute the pose error

		if(elapsed_time <= end_time)
		{
			for(int i = 0; i < 6; i++) xdot[i] = xdot_d[i] + 5.0*xe[i];		
		}
		else 
		{
			xdot.zero();							// Stop the end-effector moving
			finished = true;
		}
		
		rmrc(xdot);								// Move the hand at the desired speed
		yarp::os::Time::delay(0.002);						// Wait a bit
	}
	
	update_state();
	Ta = this->arm.getH();
	trajectory.get_state(Td, xdot_d, xddot_d, end_time);
	xe = get_pose_error(Td, Ta);
	std::cout << "Final error:" << std::endl;
	for(int i = 0; i < 3; i++) std::cout << xe[i]*1000 << " (mm)" << std::endl;
	std::cout << sqrt(xe[3]*xe[3] + xe[4]*xe[4] + xe[5]*xe[5])*180/M_PI << " (deg)" << std::endl;	
}

/******************** Basic hand motions ********************/
void ArmController::move_hand()
{
	yarp::sig::Vector speed({0.0, 0.05, 0.0, 0.0, 0.0, 0.0});
	
	double elapsed_time = 0.0;
	double start_time = yarp::os::Time::now();
	while(elapsed_time < 2.0)
	{
		elapsed_time = yarp::os::Time::now() - start_time;
		rmrc(speed*direction);
		yarp::os::Time::delay(0.002);						// Wait a bit
	}
	speed.zero();
	rmrc(speed);
	this->direction *= -1;
}
void ArmController::move_up()
{
	update_state();								// Obtain the latest joint state
	yarp::sig::Matrix T = this->arm.getH();					// Get the transform of the hand
	T(2,3) += 0.10;								// Add 10cm to the vertical direction
	move_to_pose(T);								// Run the Cartesian control loop		
}
void ArmController::move_down()
{
	update_state();								// Obtain the latest joint state
	yarp::sig::Matrix T = this->arm.getH();					// Get the transform of the hand
	T(2,3) -= 0.10;								// Subtract 10cm to the vertical direction
	move_to_pose(T);								// Run the Cartesian control loop
}
void ArmController::move_left()
{
	update_state();								// Obtain the latest joint state
	yarp::sig::Matrix T = this->arm.getH();					// Get the transform of the hand
	T(1,3) -= 0.10;								// Subtract 10cm to the vertical direction
	move_to_pose(T);								// Run the Cartesian control loop
}
void ArmController::move_right()
{
	update_state();								// Obtain the latest joint state
	yarp::sig::Matrix T = this->arm.getH();					// Get the transform of the hand
	T(1,3) += 0.10;								// Subtract 10cm to the vertical direction
	move_to_pose(T);								// Run the Cartesian control loop
}

/******************** Get and set the new joint state ********************/
bool ArmController::update_state()
{
	// TO DO: Get joint velocities
	
	if(this->encoder->getEncoders(this->q.data()))					// Get the joint angles in degrees
	{
		for(int i = 0; i < this->n; i++) this->arm.setAng(i, this->q[i]*M_PI/180);	// Set angles in radians
		return true;									// Success
	}
	else
	{
		yError() << "ArmController::update_state() : Could not obtain encoder values for " << this->name+"arm";
		return false;
	}
}

/******************** Convert SE3 matrix to a vector ********************/
yarp::sig::Vector ArmController::get_pose_error(const yarp::sig::Matrix &desired, const yarp::sig::Matrix &actual)
{
	yarp::sig::Vector error(6);
	yarp::sig::Vector axisAngle = yarp::math::dcm2axis(desired*yarp::math::SE3inv(actual));
	
	if(axisAngle[4] > M_PI)							// If angle is greater than 180 degrees...
	{
		axisAngle[4] = 2*M_PI - axisAngle[4];					// ... Take the shorter path
		for(int i = 0; i < 3; i++) axisAngle[i] *= -1;			// Flip the axis of rotation to match
	}
	
	for(int i = 0; i < 3; i++)
	{
		error[i] = desired[i][3] - actual[i][3];				// Position errors
		error[i+3] = axisAngle[4]*axisAngle[i];				// Angle * axis (if this doesn't work, try sin(0.5*angle)*axis
	}
	return error;
}

/******************** Weighting matrix for joint limit avoidance ********************/
yarp::sig::Matrix ArmController::get_joint_weighting()
{
	// NOTE: This function currently returns the INVERSE of the matrix, to save
	// on computational cost. Might need to change this in the future when the inertia
	// matrix is added.
	
	// Reference:
	// Chan, T. F., & Dubey, R. V. (1995). A weighted least-norm solution based
	// scheme for avoiding joint limits for redundant joint manipulators.
	// IEEE Transactions on Robotics and Automation, 11(2), 286-292.
	
	yarp::sig::Matrix W(this->n, this->n);					// Value to be returned
	W.eye();									// Set as identity
	double qMin, qMax;								// Store min. and max. values here
	float dwdq;									// Partial derivative
	float dMin, dMax;								// Distance from upper and lower limits
	for(int i = 0; i < this->n; i++)
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

/******************** Get the DLS of the Jacobian ********************/
yarp::sig::Matrix ArmController::dls(const yarp::sig::Matrix &J)
{
	yarp::sig::Matrix W = get_joint_weighting();					// Get the weighting matrix for joint limit avoidance (NOTE: Actually the inverse)
	
	yarp::sig::Matrix JWJt = J*W*J.transposed();					// Need this for further calcs
	
	float manipulability = sqrt(yarp::math::det(JWJt));				// Compute proximity to a singularity
	
	float damping = 0.0;
	
	if(manipulability < 0.001)							// If near singular...
	{
		damping = (1.0 - pow((manipulability/0.001),2))*0.1;			// ... increase the damping
	}

	return W*J.transposed()*yarp::math::pinvDamped(JWJt, damping);		// Return DLS inverse
}
