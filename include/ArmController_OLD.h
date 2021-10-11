#include <iostream>
#include <iCub/iKin/iKinFwd.h>							// GeoJacobian()
#include <yarp/dev/ControlBoardInterfaces.h>						// Communicates with motor controllers on the robot?
#include <yarp/dev/PolyDriver.h>							// Device driver class
#include <yarp/math/SVD.h>								// yarp::math::SVD(J,U,s,V)
#include <yarp/os/LogStream.h>							// yarp::Info() and the like
#include <yarp/os/Property.h>								// Don't know exactly what this does
#include <CartesianTrajectory.h>							// Custom trajectory class

class ArmController
{
	public:
		ArmController(	const std::string &local,				// Constructor
				const std::string &remote,
				const std::string &_name)
				:
				name(_name),						// Used for printing info
				arm(iCub::iKin::iCubArm(name))			// Construct arm object
		{
			configure_arm(local, remote);
			
			std::cout << "Number of links for " << this->name <<":" << this->arm.getN() << std::endl;
			
			print_jacobian();
		}
		
		
		// Public functions
		void move_to_position(yarp::sig::Vector &target);			// Move the arm to a desired joint position
		void move_to_pose(const yarp::sig::Matrix &Tf);			// Move the hand to a desired pose
		void close() {this->driver.close();}					// Closes the device drivers
		void print_pose();
		void move_up();
		void move_down();
		void move_left();
		void move_right();
		
		void print_jacobian();
		
	private:
	
		// Properties
		int n;									// Number of joints
	
		std::string name;							// Used to identify the object

		yarp::sig::Vector pos,vel;						// Joint state vectors for the arm
		
		iCub::iKin::iCubArm arm;
		
		// These interface with the hardware
    		yarp::dev::IControlLimits* limits;					// Joint limits?
		yarp::dev::IControlMode* mode;					// Sets the control mode of the motor
		yarp::dev::IEncoders* encoder;
		yarp::dev::IVelocityControl* controller;				// Motor-level velocity controller
		yarp::dev::PolyDriver driver;						// Device driver
		
		// Private functions	
		bool configure_arm(const std::string &local, const std::string &remote); // Connects drivers, encoders
		yarp::sig::Vector SE3toVector(const yarp::sig::Matrix &T);		// Convert SE3 matrix to vector form
		void dls(const yarp::sig::Matrix &J, yarp::sig::Matrix &invJ);	// Use DLS
		
		void update_state();							// Updates all internal state vectors
		
		
};											// Semicolon needed after class declaration

void ArmController::print_jacobian()
{
	update_state();
	yarp::sig::Matrix J = this->arm.GeoJacobian(this->pos*180/M_PI);
	std::cout <<"Here is the Jacobian:" << std::endl;
	for(int i = 0; i < J.rows(); i++)
	{
		for(int j = 0; j < J.rows(); j++)
		{
			std::cout << J(i,j) << " ";
		}
		std::cout << std::endl;
	}
}

void ArmController::update_state()
{
	this->encoder->getEncoders(this->pos.data());					// Get current encoder readings
	for(int i = 0; i < 7; i++) this->arm.setAng(i,this->pos[i]);
}
	

/******************** Move the hand up a few centimetres ********************/
void ArmController::move_up()
{
	update_state();								// Get current joint state
	yarp::sig::Matrix T = this->arm.getH();					// Get pose at current state
	T[2][3] += 0.1;								// Move up a tiny bit
	move_to_pose(T);
}

/******************** Move the hand down a few centimetres ********************/
void ArmController::move_down()
{
	update_state();								// Get current joint state
	yarp::sig::Matrix T = this->arm.getH();					// Get pose at current state
	T[2][3] -= 0.1;								// Move up a tiny bit
	move_to_pose(T);
}
/******************** Move the hand up a few centimetres ********************/
void ArmController::move_left()
{
	update_state();								// Get current joint state
	yarp::sig::Matrix T = this->arm.getH();					// Get pose at current state
	T[1][3] += 0.1;								// Move up a tiny bit
	move_to_pose(T);
}
/******************** Move the hand up a few centimetres ********************/
void ArmController::move_right()
{
	update_state();								// Get current joint state
	yarp::sig::Matrix T = this->arm.getH();					// Get pose at current state
	T[1][3] -= 0.1;								// Move up a tiny bit
	move_to_pose(T);
}



/******************** Return the DLS inverse ********************/
void ArmController::dls(const yarp::sig::Matrix &A, yarp::sig::Matrix &invA)
{
	int m = (int)A.rows();
	int n = (int)A.cols();
	
	yarp::sig::Matrix U(m,m);							// Left orthogonal matrix
	yarp::sig::Vector s(m);							// Vector of singular values
	yarp::sig::Matrix V(n,n);							// Right orthogonal matrix
	
	yarp::math::SVD(A,U,s,V);							// Get the singular value decomposition
	
	for(int i = 0; i < n; i++)
	{
		for(int j = 0; j < m; j++)
		{
			if(s(i) > 1.0e-3)
			{
				invA(i,j) = V(i,j)/s(j);				// This circumvents the off-diagonal elements for V*S^-1
			}
			else
			{
				invA(i,j) = V(i,j)*1e3;
				//yError("ArmController::dls() : Matrix is singular!");
			}
		}
	}
	invA *= U.transposed();							// Now do (V*S^-1)*U^T
}

/******************** Converts SE(3) to a 6x1 vector for feedback control ********************/
yarp::sig::Vector ArmController::SE3toVector(const yarp::sig::Matrix &T)
{
	yarp::sig::Vector x(6);							// Value to be returned
	yarp::sig::Vector axisAngle = yarp::math::dcm2axis(T);			// Get the error as axis-angle representation
	
	if(axisAngle[3] > M_PI)							// If angle is greater than 180 degrees...
	{
		axisAngle[3] = 2*M_PI - axisAngle[3];					// ... Take the shorter route...
		for(int i = 0; i < 3; i++) axisAngle[i] *= -1;			// ... and flip the axis to match
	}
	
	for(int i = 0; i < 3; i++)
	{
		x[i] 	= T[i][3];							// Translation component
		x[i+3]	= axisAngle[3]*axisAngle[i];					// If this doesn't work, try sign(0.5*angle)*axis;
	}
			
	return x;
}

/******************** Move the arm to a desired joint configuration ********************/
void ArmController::move_to_position(yarp::sig::Vector &target)
{
	// List of all local variables
	bool finished = false;
	double qMin, qMax;								// Min. and max. joint angles
	double vMin, vMax;								// Min. and max. joint speeds
	double start_time, elapsed_time;						// For the control stuff
	float dq;									// Difference in joint angles
	float end_time = 0.0;								// When the trajectory finishes
	yarp::sig::Vector ctrl(7);							// Control to send to motors
	yarp::sig::Vector pos_d(7);							// Desired joint positions
	yarp::sig::Vector vel_d(7);							// Desired joint velocities
	yarp::sig::Vector acc_d(7);							// Desired joint accelerations (unused)
	
	// Check to see if all target positions are inside the joint limits
	for(int i = 0; i < 7; i++)
	{
		this->limits->getLimits(i, &qMin, &qMax);
		if(target[i] > qMax)
		{
			yError() << "Target for joint positions is out of bounds."
			<< "Target of" << target[i] << "adjust to" << qMax - 0.1 << ".";	
			target[i] = qMax - 0.1;

		}
		else if(target[i] < qMin)
		{
			yError() << "Target for joint positions is out of bounds."
			<< "Target of" << target[i] << "adjusted to" << qMin + 0.1 << ".";
			target[i] = qMin + 0.1;
		}
	}
	
	// Figure out optimal time scaling for the trajectory
	this->encoder->getEncoders(this->pos.data());					// Get current joint positions
	for(int i = 0; i < 7; i++)
	{
		dq = target[i] - this->pos[i];					// Distance to target
		this->limits->getVelLimits(i, &vMin, &vMax);				// Get velocity limits for ith joint
		
		float dt;
		
		if(dq >= 0)	dt = (15*dq)/(8*vMax);					// Time to cover distance at max speed			
		else		dt = (15*dq)/(8*vMin);					
		
		if(dt > end_time) end_time = dt;					// Update based on maximum time
	}
	end_time *= 1.5;								// Make the time a bit longer
	
	Quintic trajectory(this->pos, target, 0, end_time);				// Create trajectory object
	
	// Run the control loop
	start_time = yarp::os::Time::now();						// Time at which control loop started
	while(!finished)
	{
		elapsed_time = yarp::os::Time::now() - start_time;			// Current loop time
		
		this->encoder->getEncoders(this->pos.data());				// Get current joint positions
		
		// Compute new joint control
		if(elapsed_time <= end_time)
		{
			trajectory.get_state(pos_d, vel_d, acc_d, elapsed_time);	// Get desired state at current time
			
			for(int i = 0; i < 7; i++)
			{
				ctrl[i] = vel_d[i]					// Feedforward term
					+ 10.0*(pos_d[i] - this->pos[i]);		// Feedback term
			}
		}
		else
		{
			ctrl.zero();							// Ensure final command is zero
			finished = true;
		}
		
		for(int i = 0; i < 7; i++) this->controller->velocityMove(i, ctrl[i]); // Send command to motors
		yarp::os::Time::delay(0.005);						// Wait a little bit
	}
}

/****************** Move the hand to a desired pose *******************/
void ArmController::move_to_pose(const yarp::sig::Matrix &target)
{

	// Here are all the local variables in this function:
	bool finished = false;
	double start_time, elapsed_time;						// Control loop timing
	float end_time = 3.0;								// End time for the trajectory
	yarp::sig::Matrix Ta(4,4);							// Actual endpoint transform
	yarp::sig::Matrix Td(4,4);							// Desired pose as a transform
	yarp::sig::Matrix J(6,7);							// Jacobian
	yarp::sig::Matrix invJ(7,6);							// Inverse Jacobian
	yarp::sig::Vector vel_d(6);							// Desired velocity
	yarp::sig::Vector acc_d(6);							// Desired acceleration
	yarp::sig::Vector xe(6);							// Pose error as a vector
	yarp::sig::Vector ctrl(7);							// Control for the motors

	update_state();								// Get current joint state (updates iKinChain object)
	
	// Construct the trajectory object
	Ta = this->arm.getH();								// Get the homogeneous transform of the endpoint
	CartesianTrajectory trajectory(Ta, target, 0.0, end_time);			// Create Cartesian trajectory
	
	//std::cout << "Start position: " << Ta(0,3) << " " <<  Ta(1,3) << " " << Ta(2,3) << std::endl;
	
	// Run the control loop
	start_time = yarp::os::Time::now();						// Time at the beginning of the control loop
	while(!finished)
	{
		elapsed_time = yarp::os::Time::now() - start_time;			// Current control time since start
		
		update_state();							// Update joint state
		
		Ta = this->arm.getH();							// Get current endpoint as transform
		
		trajectory.get_state(Td, vel_d, acc_d, elapsed_time);		// Get desired state
		
		//std::cout << "Desired position: " <<  Td(0,3) << " " << Td(1,3) << " " << Td(2,3) << std::endl;
		
		ctrl.zero();								// Zero the joint control
		
		if(elapsed_time <= end_time)
		{
			xe = SE3toVector(yarp::math::SE3inv(Ta)*Td);			// Get the error as a 6x1 vector
			
			//std::cout << "Error: " << xe[0] << " " << xe[1] << " " << xe[2] << std::endl;

		}
		else finished = true;

		for(int i = 0; i < 7; i++) this->controller->velocityMove(i,ctrl[i]);
		
		//std::cout << std::endl;
		
		yarp::os::Time::delay(0.1);
	}
}

/****************** Print out the current endpoint pose *******************/
void ArmController::print_pose()
{
	yarp::sig::Vector x = this->arm.EndEffPose(true);
	
	std::cout << "Here is the pose of the " << this->name << " arm." << std::endl;
	for(int i = 0; i < 7; i++)
	{
		std::cout << x[i] << std::endl;
	}
	
	
}
/******************** Configure the device drivers, controllers, and encoders ********************/
bool ArmController::configure_arm(const std::string &local, const std::string &remote)
{
	bool success = false;
	
	// First configure the device driver
	yarp::os::Property options;							// Object to temporarily store options
	options.put("device", "remote_controlboard");				
	options.put("local", local);							// Local port names
	options.put("remote", remote);						// Where we want to connect to

	this->driver.open(options);							// Assign the options to the driver

	if(this->driver.isValid())							// Check if requested driver is available	
	{
		yInfo() << "Successfully configured device driver for" << this->name+" arm";
		success = true;
	}			
	else
	{
		yError("Unable to configure device. Double check what devices are available.");
		success = false;
	}
	
	// Then configure the joint controllers
	if(!this->driver.view(this->controller))
	{
		yError() << "Unable to configure the controller for" << this->name+" arm.";
		this->driver.close();
		success &= false;
	}
	else if(!this->driver.view(this->mode))
	{
		yError() << "Unable to configure the control mode for" << this->name+" arm.";
		success &= false;
	}
	else
	{
		this->controller->getAxes(&this->n);					// NO. OF JOINTS IS SET HERE
		
		for(int i = 0; i < this->n; i++)
		{
			this->mode->setControlMode(i, VOCAB_CM_VELOCITY);		// Set the motor in velocity mode
			this->controller->setRefAcceleration(i,std::numeric_limits<double>::max()); // CHANGE THIS?
			this->controller->velocityMove(i,0);				// Ensure initial velocity is zero
		}
		yInfo() << "Successfully configured velocity control of" << this->name+" arm.";
		success &= true;
	}
	
	// Try and obtain the joint limits
	if(!this->driver.view(this->limits))
	{
		yError() << "Unable to obtain the joint limits for" << this->name+" arm.";
		success &=false;
	}
	else
	{
		yInfo() << "Successfully obtained limits for" << this->name+" arm.";
		success &= true;
	}
	
	// Finally, configure the encoders
	if(this->driver.view(this->encoder))
	{
		this->encoder->getAxes(&this->n);					// Get the number of axes
		this->pos.resize(this->n);						// Resize joint position vector accordingly
		
		for(int i = 0; i < 5; i++)
		{
			if(this->encoder->getEncoders(this->pos.data()))		// Get initial encoder values
			{
				yInfo() << "Successfully configured the encoder for" << this->name+" arm.";
				success &= true;
				break;							// This should exit the for-loop
			}
			if(i == 5)
			{
				yError() << "Could not obtain encoder values for" << this->name+" arm" << "in 5 attempts.";
				success &= false;
			}
			
			yarp::os::Time::delay(0.02);					// Wait a little bit and try again
		}
	}
	else
	{
		yError() << "Could not configure the encoder for" << this->name+" arm.";
		success &= false;
	}
	
	return success;
}
