#include <ArmController.h>
#include <vector>								// std::vector
#include <yarp/os/PeriodicThread.h>

class DualArmController : virtual public yarp::os::PeriodicThread
{
	public:
		DualArmController();						// Constructor
		
		void update_state();						// Update state for all objects
		void move_to_position(yarp::sig::Vector &left,		// Move the joints to a desired configuration
					yarp::sig::Vector &right);			
		void move_to_pose(const yarp::sig::Matrix &left,		// Move the hand to a desired pose
				const yarp::sig::Matrix &right,
				const double &time);			
		bool threadInit();						// Executed after start() and before run()
		void run();							// Main control loop
		void threadRelease();						// Executed after stop is called
		void close();
		yarp::sig::Vector get_positions(const std::string &which);	
	
	private:
		// These are used to regulate the arm control
		int controlMode;						// 1 = Joint, 2 = Cartesian, 3 = Hybrid
		bool controlSwitch[2];						// Switch for activating control of the arms
		double startTime, elapsedTime;				// For timing in the control
		
		// These handle arm and joint kinematics
		ArmController leftArm, rightArm;				// Handles Cartesian stuff
		MultiJointController torso;					// Low level interface for the torso joints
		
		// These are used in the Cartesian control
		yarp::sig::Matrix J;						// Jacobian for both arms
		yarp::sig::Matrix j;						// Jacobian for a single arm
		yarp::sig::Vector xdot;					// Task velocities
		
		// Functions
		yarp::sig::Matrix get_joint_weighting();			// Weight the joint motions
		yarp::sig::Matrix get_pseudoinverse(const yarp::sig::Matrix &J,
							const yarp::sig::Matrix &W);
		
		

};										// Semicolon needed after a class declaration

/******************** Move the joints to desired configuration ********************/
void DualArmController::move_to_position(yarp::sig::Vector &left, yarp::sig::Vector &right)
{
	if(isRunning()) stop();						// Stop any threads that are running
	
	if(left.size() != 0)	this->controlSwitch[0] = true;
	else			this->controlSwitch[0] = false;
	
	if(right.size() != 0)	this->controlSwitch[1] = true;
	else			this->controlSwitch[1] = false;
	
	
	if(!this->controlSwitch[0] && !this->controlSwitch[1])
	{
		yError("DualArmController::move_to_position() : Both arguments have null inputs!");
	}
	else
	{
		this->controlMode = 1;	// Use to trigger joint control mode
		if(this->controlSwitch[0]) this->leftArm.set_joint_trajectory(left);
		if(this->controlSwitch[1]) this->rightArm.set_joint_trajectory(right); 
	}
	
	start(); // Go immediately to threadInit() 
}

/******************** Move the hand to a desired configuration ********************/
void DualArmController::move_to_pose(const yarp::sig::Matrix &left, const yarp::sig::Matrix &right, const double &time)
{
	if(isRunning()) stop();						// Stop any threads that are running
	
	if(left.rows() != 0)	this->controlSwitch[0] = true;		// Turn on left arm control
	else			this->controlSwitch[0] = false;
	
	if(right.rows() != 0)	this->controlSwitch[1] = true;		// Turn on right arm control
	else			this->controlSwitch[1] = false;
	
	
	if(this->controlSwitch[0] + this->controlSwitch[1] == false)		// Both switches are off for some reason!
	{
		yError("DualArmController::move_to_position() : Both arguments have null inputs!");
	}
	else
	{
		this->controlMode = 2;						// Use to trigger Cartesian control mode
		update_state();						// This will update the kinematic chain
		if(this->controlSwitch[0]) this->leftArm.set_cartesian_trajectory(left, time); // Set new Cartesian trajectories
		if(this->controlSwitch[1]) this->rightArm.set_cartesian_trajectory(right, time);
	}
	
	start(); // Go immediately to threadInit();
}

/******************** Executed just before run() ********************/
bool DualArmController::threadInit()
{
	this->startTime = yarp::os::Time::now();				// Start the timer for the control loop
	return true;
	// Now go immediately to run()
}

/******************** MAIN CONTROL LOOP ********************/
void DualArmController::run()
{
	update_state();									// Update the joint state information
	
	this->elapsedTime = yarp::os::Time::now() - this->startTime;				// Update the time in the control loop
	
	// Solve the relevant control
	switch(this->controlMode)
	{
		case 1: // Joint control
		{
			// this->torso.joint_control(this->elapsedTime);
			if(controlSwitch[0]) this->leftArm.joint_control(this->elapsedTime);
			if(controlSwitch[1]) this->rightArm.joint_control(this->elapsedTime);
			
			break;
		}
		case 2: // Cartesian control
		{
			this->J.zero();							// Clear the Jacobian
			
			if(controlSwitch[0])							// Left arm control active
			{
				// Solve the Cartesian control part
				this->xdot.subVector(0,5) = this->leftArm.cartesian_control(this->elapsedTime);
				
				// Construct the Jacobian
				this->j = this->leftArm.get_jacobian();			// Get the Jacobian for the current state
				this->J.setSubmatrix(this->j.submatrix(0,5,0,6),0,0);	// Allocate Jacobian for arm control
				this->J.setSubmatrix(this->j.submatrix(0,5,7,9),0,14);	// Allocate the Jacobian for the torso
			}
			
			if(controlSwitch[1])							// Right arm control active
			{
				// Solve the Cartesian control part
				this->xdot.subVector(0,5) = this->rightArm.cartesian_control(this->elapsedTime);
				
				// Construct the Jacobian
				this->j = this->leftArm.get_jacobian();			// Get the Jacobian for the current state
				this->J.setSubmatrix(this->j.submatrix(0,5,0,6),6,7);	// Allocate Jacobian for arm control
				this->J.setSubmatrix(this->j.submatrix(0,5,7,9),6,14);	// Allocate the Jacobian for the torso
			}
			
			// Solve differential inverse kinematics
			yarp::sig::Matrix W = get_joint_weighting();				// NOTE: This is already the inverse!
			yarp::sig::Matrix invJ = get_pseudoinverse(J,W);			// Get the weighted pseudoinverse Jacobian
			//yarp::sig::Matrix I(17,17);
			//I.zero();
			yarp::sig::Vector qdot = invJ*xdot;// + (I - invJ*J)*rdot;
			
			// Send the commands to the joints
			if(controlSwitch[0]) this->leftArm.move_at_speed(qdot.subVector(0,6));	// Left arm joints
			if(controlSwitch[1]) this->rightArm.move_at_speed(qdot.subVector(7,13));	// Right arm joints
			this->torso.move_at_speed(qdot.subVector(14,16));				// Torso joints
			
			break;
		}
		case 3: // Something else?
		{
			break;
		}
		default: // ¯\_(ツ)_/¯
		{
			yInfo() << "Worker bees can leave.";
			yInfo() << "Even drones can fly away.";
			yInfo() << "The Queen is their slave.";
			break;
		}
	}	
	
	// Run this function repeatedly until stop() is called, then go to threadRelease();
}

/******************** Executed after stop() is called ********************/
void DualArmController::threadRelease()
{
	this->torso.move_at_speed(yarp::sig::Vector({0,0,0}));
	this->leftArm.move_at_speed(yarp::sig::Vector({0,0,0,0,0,0,0}));
	this->rightArm.move_at_speed(yarp::sig::Vector({0,0,0,0,0,0,0}));
}


/******************** Get the joint weighting matrix ********************/
yarp::sig::Matrix DualArmController::get_joint_weighting()
{
	yarp::sig::Matrix W(17,17);						// Value to be returned
	W.eye();								// Set as identity	
	
/*
	double qMin, qMax, upper, lower, range, dpdq;
	for(int i = 0; i < this->n; i++)
	{
		this->limits->getLimits(i, &qMin, &qMax);			// Get joint limits for the ith joint
		upper = qMax - this->q[i];					// Distance to the upper limit
		lower = this->q[i] - qMin;					// Distance from the lower limit
		range = qMax - qMin;						// Range between limits
		dpdq = 1/pow(upper,2) - 1/pow(lower, 2);			// Gradient
		
		if(dpdq*this->qdot[i] > 0)					// Moving toward a joint limit
		{
			W[i][i] /= (range/(upper*lower) - 4/range + 1);	// This is actually the INVERSE of the penalty
		}
	}
	
	return W; // Note: this is the INVERSE
*/
	
	return W;
}

/******************** Get the weighted pseudoinverse Jacobian ********************/
yarp::sig::Matrix DualArmController::get_pseudoinverse(const yarp::sig::Matrix &J, const yarp::sig::Matrix &W)
{
	if(J.cols() != W.rows() || W.cols() != W.rows())
	{
		yError("DualArmController::get_pseudoinverse() : Incorrect input dimensions. Expected an J (mxn) and W (nxn)");
		yarp::sig::Matrix temp(J.cols(), J.rows());
		temp.zero();
		return temp;
	}
	else
	{
		yarp::sig::Matrix invWJt = W*J.transposed();				// Assumes W is already inverted for now
		yarp::sig::Matrix A = J*invWJt;					// We want to invert this matrix
		
		yarp::sig::Matrix U(A.rows(), A.rows()), V(A.cols(), A.cols()); 	// Orthogonal matrices
		yarp::sig::Vector s(A.rows());					// List of singular values
		
		yarp::math::SVD(A,U,s,V);						// Get the singular value decomposition

		yarp::sig::Matrix invA(A.rows(), A.cols());
		invA.zero();
		for(int i = 0; i < A.rows(); i++)
		{
			for(int j = 0; j < A.cols(); j++)
			{
				if(s(j) == 0);	//	invA(i,J) += 0			// Ignore singular directions
				else if(s(j) < 1e-03)	invA(i,j) += V(i,j)*1e03;	// Damp near-singular directions
				else			invA(i,j) += V(i,j)/s(j);	// Proper inversion of the first half
			}
		}
		
		return invWJt*invA*U.transposed();					// Return the complete inversion
	}
}

/******************** Get and set new joint state information ********************/
void DualArmController::update_state()
{
	this->torso.get_encoder_values();						// Read encoders on torso joints
	this->leftArm.get_encoder_values();
	this->rightArm.get_encoder_values();
	
	yarp::sig::Vector dof(10);							// We need to put this in to the iCubArm objects
	
	for(int i = 0; i < 3; i++) dof[i] = torso.q[2-i];				// Torso joint order is opposite to the arms
	
	for(int i = 0; i < 7; i++) dof[i+3] = this->leftArm.q[i];			// Assign the left arm joint angles to the stack
	this->leftArm.set_angles(dof);
	
	for(int i = 0; i < 7; i++) dof[i+3] = this->rightArm.q[i];			// Assign the right arm joint angles to the stack
	this->rightArm.set_angles(dof);
}

/******************** Constructor ********************/
DualArmController::DualArmController() : yarp::os::PeriodicThread(0.01)
{
	// Resize and set matrices accordingly
	this->J.resize(12,17);
	this->J.zero();
	
	// Resize vectors
	this->xdot.resize(12);
	
	// Configure the device drivers for different parts of the robot
	this->leftArm.configure("/local/left", "/icubSim/left_arm", "left");
	this->rightArm.configure("/local/right", "/icubSim/right_arm", "right");
	this->torso.configure_drivers("/local/torso", "/icubSim/torso", "torso", 3);
}

/******************** Close the device drivers ********************/
void DualArmController::close()
{
	this->leftArm.close();
	this->rightArm.close();
	this->torso.close();
}

/******************** Get the joint positions for a single arm ********************/
yarp::sig::Vector DualArmController::get_positions(const std::string &which)
{
	if(which.compare("left"))		return this->leftArm.q;
	else if(which.compare("right"))	return this->rightArm.q;
	else
	{
		yError("DualArmController::get_positions() : Expected string input as 'left' or 'right'.");
		return yarp::sig::Vector({0,0,0,0,0,0,0});
	}
}
