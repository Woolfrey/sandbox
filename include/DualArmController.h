#include <iostream>							// For debugging
#include <ArmController.h>						// Handles forward kinematics
#include <yarp/os/PeriodicThread.h>					// Control loop

/*
	THINGS TO FIX:
		- No switch to deactivate torso control - could be a problem
		  with null inputs.
		  
		- Need to set a variable for the control frequency
*/

class DualArmController : public yarp::os::PeriodicThread
{
	public:
		DualArmController();					// Constructor
		
		void close();						// Close the device drivers
		void move_to_position(yarp::sig::Vector &left, yarp::sig::Vector &right, yarp::sig::Vector &mid);
		void move_lateral(const double &distance);
		void move_vertical(const double &distance);
		void move_in_out(const double &distance);
		void move_horizontal(const double &distance);						
		void print_pose(const std::string &which);
		void update_state();					// Update the current joint state
			
	private:
		// Variables
		bool leftControl = false;
		bool rightControl = false;
		double elapsedTime, startTime;				// Used to regulate the control loop
		int controlSpace = 100;					// 1 = Joint, 2 = Cartesian
		int nullSpaceTask = 0;					// 0 = Nothing, 1 = Manipulability, 2 = Stiffness
		yarp::sig::Vector xdot, q, qdot, qdot_R, qdot_N, qdot_d, vMin, vMax;
		yarp::sig::Matrix J, JL, JR, W, I, N, dJdq;
						
		// Objects
		ArmController leftArm, rightArm;			// Handles kinematics for the arms
		MultiJointController torso;				// Handles joint communication and control for torso
		
		// Functions
		double trace(const yarp::sig::Matrix &A);		// Get the trace of the matrix
		void get_joint_weighting();				// For joint limit avoidance in RMRC
		void scale_vector(yarp::sig::Vector &input, const yarp::sig::Vector ref); // Scale a vector based on speed limits
		void update_speed_limits();				// Get the min. and max. velocity for each joint
		yarp::sig::Matrix get_inverse(const yarp::sig::Matrix &A); // Invert a matrix. Add damping as necessary
		yarp::sig::Vector get_manip_grad(const std::string &which, const yarp::sig::Matrix &Jacobian);
		
		// Control thread functions
		bool threadInit();					// Executed after start() and before run()
		void run();						// Main control loop
		void threadRelease();					// Executed after stop is called
			
};									// Semicolon needed after class declaration

void DualArmController::print_pose(const std::string &which)
{
	update_state();
	if(which == "left")
	{
		yarp::sig::Matrix H = this->leftArm.get_pose();
		yInfo("Here is the pose for the left hand:");
		std::cout << H.toString() << std::endl;
	}
	else if(which == "right")
	{
		yarp::sig::Matrix H = this->rightArm.get_pose();
		yInfo("Here is the pose for the right hand:");
		std::cout << H.toString() << std::endl;
	}
}

/******************** Constructor ********************/
DualArmController::DualArmController() : yarp::os::PeriodicThread(0.01)
{
	// Resize vectors
	this->xdot.resize(12);						// Task velocity vector
	this->qdot.resize(17);						// Joint velocity vector for both arms
	this->qdot_R.resize(17);					// Range space task
	this->qdot_N.resize(17);					// Null space task
	this->q.resize(10);						// Joint position for a single kinematic chain
	this->qdot_d.resize(17);
	this->vMin.resize(17);						// Minimum speed for the joints
	this->vMax.resize(17);						// Maximum speed for the joints
		
	// Resize matrices
	this->J.resize(12,17);						// Jacobian for both arms
	this->W.resize(17,17);						// Weighting matrix for RMRC
		this->W.eye();
	this->I.resize(17,17);						// Identity matrix
		this->I.eye();
	this->N.resize(17,17);						// Null space projection matrix	
	this->dJdq.resize(12,17);					// Partial derivative of the Jacobian			

	// Configure the device drivers for different parts of the robot
	this->leftArm.configure("/local/left", "/icubSim/left_arm", "left");
	this->rightArm.configure("/local/right", "/icubSim/right_arm", "right");
	this->torso.configure_drivers("/local/torso", "/icubSim/torso", "torso", 3);
}

/******************** Close the device drivers ********************/
void DualArmController::close()
{
	stop();									// Stop any control threads first
	this->leftArm.close();
	this->rightArm.close();
	this->torso.close();
}

/******************** Get and set new joint state information ********************/
void DualArmController::update_state()
{
	// Read encoder values from the joints
	bool success	 = this->torso.read_encoders();
	success 	&= this->leftArm.read_encoders();
	success 	&= this->rightArm.read_encoders();
	
	if(success)
	{
		yarp::sig::Vector temp = this->torso.get_joint_positions();	// Get the joint positions from the torso
		for(int i = 0; i < 3; i++) this->q[i] = temp[2-i];		// Assign torso joints (in reverse order)
		
		this->q.setSubvector(3, this->leftArm.get_joint_positions());	// Get the joint positions from the left arm
		this->leftArm.set_joint_angles(q);				// Assign them to the iCubArm object
		
		this->q.setSubvector(3, this->rightArm.get_joint_positions());	// Overwrite with the right arm joints
		this->rightArm.set_joint_angles(q);				// Assign them to the iCubArm object
		
	}
	else	close();							// There was a problem, so shut down the robot
}

/******************** Move the joints to desired configuration ********************/
void DualArmController::move_to_position(yarp::sig::Vector &left, yarp::sig::Vector &right, yarp::sig::Vector &mid)
{
	if(isRunning()) stop();							// Stop any threads that are running
	
	if(left.size() == 0)	this->leftControl = false;			// Null input, so don't move
	else			this->leftControl = true;
	
	if(right.size() == 0)	this->rightControl = false;			// Null input, so don't move
	else			this->rightControl = true;
	
	if(!this->leftControl && !this->rightControl && mid.size() == 0)
	{
		yError("DualArmController::move_to_position() : Both arguments have null inputs!");
	}
	else
	{
		this->controlSpace = 1;						// Switch case for control
		
		// Generate new joint space trajectories internally
		this->torso.set_joint_trajectory(mid);
		if(this->leftControl)	this->leftArm.set_joint_trajectory(left);
		if(this->rightControl)	this->rightArm.set_joint_trajectory(right);
	}
	
	start(); // Go immediately to threadInit() 
}

/******************** Cartesian Control Functions ********************/
void DualArmController::move_lateral(const double &distance)
{
	if(isRunning()) stop();							// Stop any control threads
	
	this->controlSpace = 2;							// Switch case for Cartesian control
	
	// Compute the left arm control
	yarp::sig::Matrix H = this->leftArm.get_pose();				// Get the pose of the left hand
	H[1][3] -= distance;							// OPPOSITE GLOBAL FRAME
	this->leftArm.set_cartesian_trajectory(H, 2.0);				// Set the internal trajectory object
	this->leftControl = true;
	
	// Compute the right arm control
	H = this->rightArm.get_pose();
	H[1][3] -= distance;
	this->rightArm.set_cartesian_trajectory(H, 2.0);
	this->rightControl = true;
	
	start(); // Go immediately to initThread();
}
void DualArmController::move_vertical(const double &distance)
{
	if(isRunning()) stop();							// Stop any control threads
	
	this->controlSpace = 2;							// Switch case for Cartesian control
	
	// Compute the left arm control
	yarp::sig::Matrix H = this->leftArm.get_pose();				// Get the pose of the left hand
	H[2][3] += distance;							// Offset the z-position
	this->leftArm.set_cartesian_trajectory(H, 2.0);				// Set the internal trajectory object
	this->leftControl = true;
	
	// Compute the right arm control
	H = this->rightArm.get_pose();
	H[2][3] += distance;							// Offset the z position
	this->rightArm.set_cartesian_trajectory(H, 2.0);
	this->rightControl = true;
	
	start(); // Go immediately to initThread();
}
void DualArmController::move_in_out(const double &distance)
{
	if(isRunning()) stop();							// Stop any control threads
	
	this->controlSpace = 2;							// Switch case for Cartesian control
	
	// Compute the left arm control
	yarp::sig::Matrix H = this->leftArm.get_pose();				// Get the pose of the left hand
	H[1][3] += distance;							// Offset the y-position
	this->leftArm.set_cartesian_trajectory(H, 2.0);				// Set the internal trajectory object
	this->leftControl = true;
	
	// Compute the right arm control
	H = this->rightArm.get_pose();
	H[1][3] -= distance;
	this->rightArm.set_cartesian_trajectory(H, 2.0);
	this->rightControl = true;
	
	start(); // Go immediately to initThread();
}
void DualArmController::move_horizontal(const double &distance)
{
	if(isRunning()) stop();							// Stop any control threads
	
	this->controlSpace = 2;							// Switch case for Cartesian control
	
	// Compute the left arm control
	yarp::sig::Matrix H = this->leftArm.get_pose();				// Get the pose of the left hand
	H[0][3] -= distance;							// Offset the y-position
	this->leftArm.set_cartesian_trajectory(H, 2.0);				// Set the internal trajectory object
	this->leftControl = true;
	
	// Compute the right arm control
	H = this->rightArm.get_pose();
	H[0][3] -= distance;
	this->rightArm.set_cartesian_trajectory(H, 2.0);
	this->rightControl = true;
	
	start(); // Go immediately to initThread();
}

/******************** Executed after start() and before run() ********************/
bool DualArmController::threadInit()
{
	this->startTime = yarp::os::Time::now();				// Start timer for the control loop
	return true;
}

/******************** MAIN CONTROL LOOP ********************/
void DualArmController::run()
{
	update_state();								// Update the joint state information
	
	this->elapsedTime = yarp::os::Time::now() - this->startTime;		// Update the time in the control loop
	
	// Solve the relevant control
	switch(this->controlSpace)
	{
		case 1: // Joint control
		{
			this->torso.joint_control(this->elapsedTime);
			if(this->leftControl)	this->leftArm.joint_control(this->elapsedTime);
			if(this->rightControl)	this->rightArm.joint_control(this->elapsedTime);
			break;
		}
		case 2: // Cartesian control
		{
			this->J.zero();							// Clear the Jacobian
			this->xdot.zero();						// Clear the task vector
			
			// Left hand control component
			if(this->leftControl)						// Left arm control active
			{	
				// Solve the Cartesian control part
				this->xdot.setSubvector(0,this->leftArm.get_cartesian_control(this->elapsedTime));
				
				// Construct the Jacobian
				this->JL = this->leftArm.get_jacobian();
				this->J.setSubmatrix(this->JL,0,0);			// Effect on left hand
			}
			
			// Right hand control component
			if(this->rightControl)						// Right arm control active
			{
				// Solve the Cartesian control part
				this->xdot.setSubvector(6,this->rightArm.get_cartesian_control(this->elapsedTime));
				
				// Construct the Jacobian
				this->JR = this->rightArm.get_jacobian();
				this->J.setSubmatrix(this->JR.submatrix(0,5,0,2),6,0);	// Assign torso effect for right hand
				this->J.setSubmatrix(this->JR.submatrix(0,5,3,9),6,10);	// Assign arm effect for the right hand
			}
				
			// Solve differential inverse kinematics
			get_joint_weighting();						// NOTE: This is already the inverse!
			
			yarp::sig::Matrix invWJt = this->W*this->J.transposed();	// This makes calcs a little easier
			yarp::sig::Matrix invJ = invWJt*get_inverse(this->J*invWJt); // Get the pseudoinverse
			
			this->qdot_R = invJ*this->xdot;					// Range space task
			
			update_speed_limits();						// Update for joint limit avoidance
			
			scale_vector(this->qdot_R, this->qdot_R);			// Scale range space vector to obey limits
			
			this->nullSpaceTask = 1;

			switch(this->nullSpaceTask)
			{
				case 0:							// Minimum velocity?
				{
					this->qdot_d.zero();
					break;
				}
				case 1:							// Optimise manipulability
				{
					// Get the gradient for the left arm
					yarp::sig::Vector temp = get_manip_grad("left", this->JL);
					for(int i = 0; i < 3; i++) this->qdot_d[i]	= 0.5*temp[i];
					for(int i = 0; i < 7; i++) this->qdot_d[i+3]	= temp[i+3];
					
					// Get the gradient for the right arm
					temp = get_manip_grad("right", this->JR);
					for(int i = 0; i < 3; i++) this->qdot_d[i]	+= 0.5*temp[i];
					for(int i = 0; i < 7; i++) this->qdot_d[i+10]	= temp[i+3];
					
					this->qdot_d *= 2.0;				// Scale it
					
					break;
				}
				default:
				{
					yInfo("Worker bees can leave.");
					yInfo("Even drones can fly away.");
					yInfo("The Queen is their slave.");
				}
			}	
			
			this->N = this->I - invJ*this->J;				// Null space projection matrix
			this->qdot_N = this->N*this->qdot_d;				// Project the desired velocity on the null space
			
			this->qdot = this->qdot_R + this->qdot_N;			// Combine the range space and null space task
			
			scale_vector(this->qdot_N, this->qdot);				// Scale null space task to obey joint limits

			// Send the commands to the motors
			this->torso.move_at_speed(yarp::sig::Vector({this->qdot[2],
								     this->qdot[1],
								     this->qdot[0]})); // IN REVERSE ORDER
								     
			this->leftArm.move_at_speed(this->qdot.subVector(3,9));
			this->rightArm.move_at_speed(this->qdot.subVector(10,16));

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

void DualArmController::threadRelease()
{
	// Ensure the last command is zero velocity
	this->torso.move_at_speed(yarp::sig::Vector({0,0,0}));
	this->leftArm.move_at_speed(yarp::sig::Vector({0,0,0,0,0,0,0}));
	this->rightArm.move_at_speed(yarp::sig::Vector({0,0,0,0,0,0,0}));
	
	// Set the triggers as false to prevent unwanted motion
	this->leftControl = false;
	this->rightControl = false;
}

/******************** Get the joint weighting matrix ********************/
void DualArmController::get_joint_weighting()
{
	// Note to self: It's easier to compute the penalty function in
	// the MultiJointController class and send the result here.
	
	for(int i = 0; i < 3; i++) this->W[i][i] = this->torso.get_joint_weight(2-i);	// Torso joints are backwards in the kinematic chain
	for(int i = 0; i < 7; i++)
	{
		this->W[i+3][i+3]	= this->leftArm.get_joint_weight(i);
		this->W[i+10][i+10]	= this->rightArm.get_joint_weight(i);	
	}
}

/******************** Get the weighted pseudoinverse Jacobian ********************/
yarp::sig::Matrix DualArmController::get_inverse(const yarp::sig::Matrix &A)
{
	yarp::sig::Matrix U(A.rows(), A.cols()), V(A.cols(), A.rows());			// Orthogonal matrices
	yarp::sig::Vector s(A.rows());							// Vector of singular values
	yarp::math::SVD(A, U, s, V);							// Get the SVD
	
	yarp::sig::Matrix invA(A.cols(), A.rows());					// To be returned
	invA.zero();
	
	for(int i = 0; i < A.rows(); i++)
	{
		for(int j = 0; j < A.cols(); j++)
		{
			if(s(j) == 0);							// Ignore singular directions
			else if(s(j) < 1e-03)	invA(i,j) += V(i,j)*1e03;
			else			invA(i,j) += V(i,j)/s(j);
		}
	}
	
	return invA*U.transposed();							// Complete the inversion and return	
}

void DualArmController::update_speed_limits()
{
	for(int i = 0; i < 3; i++)
	{
		this->torso.get_speed_limits(2-i, this->vMin[i], this->vMax[i]);	// Torso joints are backwards
	}
	for(int i = 0; i < 7; i++)
	{
		this->leftArm.get_speed_limits(i, this->vMin[i+3], this->vMax[i+3]);
		this->rightArm.get_speed_limits(i,this->vMin[i+10],this->vMax[i+10]);
	}
}

void DualArmController::scale_vector(yarp::sig::Vector &input, const yarp::sig::Vector ref)
{
	double s = 1.0;
	for(int i = 0; i < ref.size(); i++)
	{
		if(ref[i] >= this->vMax[i] && this->vMax[i]/ref[i] < s)			// If ref is above limits AND largest observed so far...
		{
			s = 0.99*this->vMax[i]/ref[i];					// ... update the scalar
		}
		else if(ref[i] < this->vMin[i] && this->vMin[i]/ref[i] < s)		// Else if ref is below limits and largest observed so far...
		{
			s = 0.99*this->vMin[i]/ref[i];					// ... update the scalar
		}
	}
	input *= s;									// Return the scaled vector
}

double DualArmController::trace(const yarp::sig::Matrix &A)
{
	double t = 0;									// Value to be returned
	int m = std::min(A.cols(), A.rows());						// Get the minimum between rows, columns
	for(int i = 0; i < m ; i++) t += A[i][i];					// Add the diagonal elements
	return t;									// Return
}

yarp::sig::Vector DualArmController::get_manip_grad(const std::string &which, const yarp::sig::Matrix &Jacobian)
{
	// Variables to be used
	yarp::sig::Matrix JJt = Jacobian*Jacobian.transposed();				// Makes calcs a little faster
	double mu = sqrt(yarp::math::det(JJt));						// Measure of manipulability
	yarp::sig::Matrix invJJt = get_inverse(JJt);					// Makes calcs a little faster
	yarp::sig::Vector grad(10);							// Value to be returned
	
	// Prepare the iKinChain object to compute the Hessian
	if(which == "left")		this->leftArm.prep_for_hessian();
	else if(which == "right")	this->rightArm.prep_for_hessian();
	else				yError("DualArmController::get_manip:grad() : Expected 'left' or 'right' as an argument.");
	
	// Compute the gradient vector
	for(int i = 0; i < 10; i++)
	{
		if(which == "left")		this->dJdq = this->leftArm.get_partial_jacobian(i);
		else if(which == "right")	this->dJdq = this->rightArm.get_partial_jacobian(i);
		else yError("DualArmController::get_manip:grad() : Expected 'left' or 'right' as an argument.");
		
		grad[i] = mu*trace(dJdq*Jacobian.transposed()*invJJt);
	}
	
	return grad;
}
