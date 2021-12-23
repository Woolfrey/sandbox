#include <iostream>							// For debugging
#include <ArmCtrl.h>							// Handles forward kinematics
#include <yarp/os/PeriodicThread.h>					// Control loop

class DualArmCtrl : public yarp::os::PeriodicThread
{
	public:
		// Constructors(s)
		DualArmCtrl(const double &freq);			// Constructor
		
		// General Functions
		bool update_state();					// Update the position and velocity of the joints
		void close();						// Close the device drivers
		void print_pose(const std::string &which);		// Print the pose of the specified hand
	
		// Get Functions
		yarp::sig::Matrix get_hand_pose(const std::string &which);
		
		// Cartesian Control Functions
		void move_to_position(yarp::sig::Vector &left,		// Move the joints to a desired position
				yarp::sig::Vector &right,
				yarp::sig::Vector &mid);
				
		void move_to_pose(const yarp::sig::Matrix &left,
				const yarp::sig::Matrix &right);
		
		void set_redundant_task(const unsigned int &num, const double &scalar);
		
	private:
		// Variables
		bool leftControl = false;
		bool rightControl = false;
		double elapsedTime, startTime;				// Used to regulate the control loop
		double alpha = 1;					// Gain on the redundant task for GPM
		int controlSpace = 100;					// 1 = Joint, 2 = Cartesian
		int nullSpaceTask = 0;					// 0 = Nothing, 1 = Manipulability, 2 = Stiffness
		yarp::sig::Vector xdot, q, qdot, qdot_R, qdot_N, qdot_d, vMin, vMax, gradL, gradR;
		yarp::sig::Matrix J, JL, JR, W, I, N, dJdq;
						
		// Objects
		ArmCtrl leftArm, rightArm;				// Handles kinematics for the arms
		JointCtrl torso;					// Handles joint communication and control for torso
		
		// Control thread functions
		bool threadInit();					// Executed after start() and before run()
		void run();						// Main control loop
		void threadRelease();					// Executed after stop is called
			
		// Joint Limit Avoidance Functions
		void get_joint_weighting();				// For joint limit avoidance in RMRC
		void scale_vector(yarp::sig::Vector &input, const yarp::sig::Vector ref); // Scale a vector based on speed limits
		void update_speed_limits();				// Get the min. and max. velocity for each joint		
		
		// Math Functions
		double dot_product(const yarp::sig::Vector &a, const yarp::sig::Vector &b);
		double trace(const yarp::sig::Matrix &A);			// Get the trace of the matrix
		yarp::sig::Matrix get_inverse(const yarp::sig::Matrix &A); 	// Invert a matrix. Add damping as necessary		
		
		// Gradient Vectors
		yarp::sig::Vector get_manip_grad(const std::string &which, const yarp::sig::Matrix &Jacobian);
		yarp::sig::Vector get_force_grad(const std::string &which, const yarp::sig::Matrix &Jacobian, const yarp::sig::Vector &wrench);
			
};									// Semicolon needed after class declaration

/******************** Constructor ********************/
DualArmCtrl::DualArmCtrl(const double &freq) : yarp::os::PeriodicThread(1/freq)
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
	this->gradL.resize(10);
	this->gradR.resize(10);						// Gradient vectors
		
	// Resize matrices
	this->J.resize(12,17);						// Jacobian for both arms
	this->W.resize(17,17);						// Weighting matrix for RMRC
		this->W.eye();
	this->I.resize(17,17);						// Identity matrix
		this->I.eye();
	this->N.resize(17,17);						// Null space projection matrix	
	this->dJdq.resize(6,10);					// Partial derivative of the Jacobian			

	// Configure the device drivers for different parts of the robot
	this->leftArm.configure("/local/left", "/icubSim/left_arm", "left");
	this->rightArm.configure("/local/right", "/icubSim/right_arm", "right");
	this->torso.configure_drivers("/local/torso", "/icubSim/torso", "torso", 3);
	
	// Update the frequency in the JointCtrl objects
	this->leftArm.set_frequency(freq);
	this->rightArm.set_frequency(freq);
	this->torso.set_frequency(freq);
	
	// Set a new base reference for the left arm
	yarp::sig::Matrix T = yarp::math::rpy2dcm(yarp::sig::Vector({0,0,M_PI}));	// Reverse rotation of origin to torso
	T[2][3] = 0.65;									// Height of the torso
	this->leftArm.set_base_pose(T*this->leftArm.get_base_pose());			// Set a new first pose
	
	// Set a new base reference for the right arm
	this->rightArm.set_base_pose(T*this->rightArm.get_base_pose());
	

	update_state();									// Update the joint state
}

/******************** Get and set new joint state information ********************/
bool DualArmCtrl::update_state()
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
	
	return	success;
}

/******************** Close the device drivers ********************/
void DualArmCtrl::close()
{
	stop();									// Stop any control threads first
	this->leftArm.close();
	this->rightArm.close();
	this->torso.close();
}

/******************** Print the pose for one of the hands ********************/
void DualArmCtrl::print_pose(const std::string &which)
{
	update_state();
	if(which == "left")
	{
		yInfo("Here is the pose for the left hand:");
		std::cout << this->leftArm.get_pose().toString() << std::endl;
	}
	else if(which == "right")
	{
		yInfo("Here is the pose for the right hand:");
		std::cout << this->rightArm.get_pose().toString() << std::endl;
	}
	else if(which == "base")
	{
		yInfo("Here is the pose for the base:");
		std::cout << this->leftArm.get_base_pose().toString() << std::endl;
	}
}

/******************** Get the pose of the hand ********************/
yarp::sig::Matrix DualArmCtrl::get_hand_pose(const std::string &which)
{
	if(which == "left")		return this->leftArm.get_pose();
	else if(which == "right")	return this->rightArm.get_pose();
	else
	{
		yError("DualArmCtrl::get_hand_pose() : Expected 'left' or 'right' as an argument!");
		yarp::sig::Matrix temp(4,4);
		return temp.eye();
	}
}	

/******************** Move the joints to desired configuration ********************/
void DualArmCtrl::move_to_position(yarp::sig::Vector &left, yarp::sig::Vector &right, yarp::sig::Vector &mid)
{
	// NOTE TO SELF: Maybe I can change the logic here so that the arm only stops moving
	// once all the settings are made?
	
	if(isRunning()) stop();							// Stop any threads that are running
	
	if(left.size() == 0)	this->leftControl = false;			// Null input, so don't move
	else			this->leftControl = true;
	
	if(right.size() == 0)	this->rightControl = false;			// Null input, so don't move
	else			this->rightControl = true;
	
	if(!this->leftControl && !this->rightControl && mid.size() == 0)
	{
		yError("DualArmCtrl::move_to_position() : All three arguments have null inputs!");
	}
	else
	{
		this->controlSpace = 1;						// Switch case for control
		
		// Generate new joint space trajectories internally
		this->torso.set_joint_trajectory(mid);
		if(this->leftControl)	this->leftArm.set_joint_trajectory(left);
		if(this->rightControl)	this->rightArm.set_joint_trajectory(right);
		
		start(); // Go immediately to threadInit() 
	}
}

/******************** Move the hand(s) to a desired pose ********************/
void DualArmCtrl::move_to_pose(const yarp::sig::Matrix &left, const yarp::sig::Matrix &right)
{
	if(isRunning())	stop();							// Stop any threads that are running
	
	if(left.rows() == 0)	this->leftControl = false;
	else			this->leftControl = true;
	
	if(right.rows() == 0)	this->rightControl = false;
	else			this->rightControl = true;
	
	if(!this->leftControl && !this->rightControl)
	{
		yError("DualArmCtrl::move_to_pose() : Both arguments have null inputs!");
	}
	else
	{
		this->controlSpace = 2;						// Switch case for control
		
		// Generate new Cartesian space trajectories internally
		if(this->leftControl)	this->leftArm.set_cartesian_trajectory(left, 3.0);
		if(this->rightControl)	this->rightArm.set_cartesian_trajectory(right, 3.0);
		
		start(); // Go immediately to threadInit()
	}
}

/******************** Set the null space task ********************/
void DualArmCtrl::set_redundant_task(const unsigned int &num, const double &scalar)
{
	this->nullSpaceTask = num;
	this->alpha = scalar;
}

/******************** Executed after start() and before run() ********************/
bool DualArmCtrl::threadInit()
{
	this->startTime = yarp::os::Time::now();				// Start timer for the control loop
	return true;
}

/******************** MAIN CONTROL LOOP ********************/
void DualArmCtrl::run()
{
	update_state();								// Update the joint state information
	
	this->elapsedTime = yarp::os::Time::now() - this->startTime;		// Update the time in the control loop
	
	// Solve the relevant control
	switch(this->controlSpace)
	{
		case 1: // Joint control
		{
			this->torso.track_joint_trajectory(this->elapsedTime);
			if(this->leftControl)	this->leftArm.track_joint_trajectory(this->elapsedTime);
			if(this->rightControl)	this->rightArm.track_joint_trajectory(this->elapsedTime);
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
			yarp::sig::Matrix invJ = invWJt*get_inverse(this->J*invWJt);	// Get the pseudoinverse
			this->qdot_R = invJ*this->xdot;					// Range space task
			
			switch(this->nullSpaceTask)
			{
				case 1:
				{
					this->gradL = get_manip_grad("left", this->JL);
					this->gradR = get_manip_grad("right", this->JR);
					break;
				}
				case 2:
				{
					yarp::sig::Vector force({0, 0, 1, 0, 0, 0});
					this->gradL = get_force_grad("left", this->JL, force);
					this->gradR = get_force_grad("right", this->JR, force);
					break;
				}
				default:
				{
					this->gradL.zero();
					this->gradR.zero();
					break;
				}
			}
			
			for(int i = 0; i < 7; i++)
			{
				if(i < 3) this->qdot[i] = (this->gradL[i] + this->gradR[i])/2;
				this->qdot_d[i+3]  = this->gradL[i+3];
				this->qdot_d[i+10] = this->gradR[i+3];
			}
			this->qdot_d *= this->alpha;					// Scale the gradient
			
			// Project the redundant task on the null space and add it
			this->N = this->I - invJ*this->J;				// Null space projection matrix
			this->qdot_N = this->N*this->qdot_d;				// Project the desired velocity on the null space
			this->qdot = this->qdot_R + this->qdot_N;			// Combine the range space and null space task
			
			// Scale the vectors so they remain within joint limits
			update_speed_limits();						// Get new limits based on current state
			scale_vector(this->qdot_R, this->qdot_R);			// Scale range space vector to obey limits
			scale_vector(this->qdot_N, this->qdot);				// Scale null space task to obey joint limits
			this->qdot = this->qdot_R + this->qdot_N;			// Recombine the range and null vectors after scaling

			// Send the commands to the motors
			this->torso.move_at_speed(yarp::sig::Vector({this->qdot[2], this->qdot[1], this->qdot[0]})); // IN REVERSE ORDER
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

/******************** Executed after stop() is called ********************/
void DualArmCtrl::threadRelease()
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
void DualArmCtrl::get_joint_weighting()
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

/******************** Scale a task vector to remain within joint limits ********************/
void DualArmCtrl::scale_vector(yarp::sig::Vector &input, const yarp::sig::Vector ref)
{
	double s = 1.0;
	for(int i = 0; i < ref.size(); i++)
	{
		if(ref[i] >= this->vMax[i] && this->vMax[i]/ref[i] < s)			// If ref is above limits AND largest observed so far...
		{
			s = 0.99*this->vMax[i]/ref[i];					// ... update the scalar
		}
		else if(ref[i] < this->vMin[i] && this->vMin[i]/ref[i] < s)		// Else if ref is below limits AND largest observed so far...
		{
			s = 0.99*this->vMin[i]/ref[i];					// ... update the scalar
		}
	}
	input *= s;									// Return the scaled vector
}

/******************** Update the minimum and maximum permissable joint velocities ********************/
void DualArmCtrl::update_speed_limits()
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

/******************** Get the dot product between 2 vectors ********************/
double DualArmCtrl::dot_product(const yarp::sig::Vector &a, const yarp::sig::Vector &b)
{
	if(a.size() != b.size())
	{
		yError("DualArmCtrl::dot_product() : Vectors are not of equal length!");
		return 0;
	}
	else
	{
		double temp = 0.0;
		for(int i = 0; i < a.size(); i++)
		{
			temp += a[i]*b[i];
		}
		return temp;
	}
}

/******************** Compute the trace of a matrix ********************/
double DualArmCtrl::trace(const yarp::sig::Matrix &A)
{
	double t = 0;									// Value to be returned
	int m = std::min(A.cols(), A.rows());						// Get the minimum between rows, columns
	for(int i = 0; i < m ; i++) t += A[i][i];					// Add the diagonal elements
	return t;									// Return
}

/******************** Get the inverse of a matrix ********************/
yarp::sig::Matrix DualArmCtrl::get_inverse(const yarp::sig::Matrix &A)
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
			else if(s(j) < 1e-05)	invA(i,j) += V(i,j)*1e05;
			else			invA(i,j) += V(i,j)/s(j);
		}
	}
	
	return invA*U.transposed();							// Complete the inversion and return	
}

/******************** Get the gradient of the measure of manipulability ********************/
yarp::sig::Vector DualArmCtrl::get_manip_grad(const std::string &which, const yarp::sig::Matrix &Jacobian)
{
	// Variables to be used
	yarp::sig::Matrix JJt = Jacobian*Jacobian.transposed();				// Makes calcs a little faster
	double mu = sqrt(yarp::math::det(JJt));						// Measure of manipulability
	yarp::sig::Matrix invJJt = get_inverse(JJt);					// Makes calcs a little faster
	yarp::sig::Vector grad(10);							// Value to be returned
	
	// Prepare the iKinChain object to compute the Hessian
	if(which == "left")		this->leftArm.prep_for_hessian();
	else if(which == "right")	this->rightArm.prep_for_hessian();
	else				yError("DualArmCtrl::get_manip:grad() : Expected 'left' or 'right' as an argument.");
	
	// Compute the gradient vector
	for(int i = 0; i < 10; i++)
	{
		if(which == "left")		this->dJdq = this->leftArm.get_partial_jacobian(i);
		else if(which == "right")	this->dJdq = this->rightArm.get_partial_jacobian(i);

		grad[i] = mu*trace(this->dJdq*Jacobian.transposed()*invJJt);
	}
	return grad;
}

/******************** Get the gradient for static joint torques ********************/
yarp::sig::Vector DualArmCtrl::get_force_grad(const std::string &which, const yarp::sig::Matrix &Jacobian, const yarp::sig::Vector &wrench)
{
	// Prepare the iKinChain object to compute the Hessian
	if(which == "left")		this->leftArm.prep_for_hessian();
	else if(which == "right")	this->rightArm.prep_for_hessian();
	else				yError("DualArmCtrl::get_force_grad() : Expected 'left' or 'right' as an argument.");

	yarp::sig::Vector Jtw = Jacobian.transposed()*wrench;				// Makes calcs a little faster
	yarp::sig::Vector grad(10);							// Value to be returned
	
	for(int i = 0; i < 10; i++)
	{
		if(which == "left")		this->dJdq = this->leftArm.get_partial_jacobian(i);
		else if(which == "right")	this->dJdq = this->leftArm.get_partial_jacobian(i);
		
		grad[i] = dot_product(this->dJdq.transposed()*wrench, Jtw);
	}
	return grad;
}

/******************** Cartesian Control Functions ********************
void DualArmCtrl::move_lateral(const double &distance)
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
void DualArmCtrl::move_vertical(const double &distance)
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
void DualArmCtrl::move_in_out(const double &distance)
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
void DualArmCtrl::move_horizontal(const double &distance)
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
*/

