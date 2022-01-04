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
				
		void move_to_pose(const yarp::sig::Matrix &left,	// Move the hands to a desired pose
				const yarp::sig::Matrix &right);
				
		void translate(const yarp::sig::Vector &left,		// Move the hands by a given amount
				const yarp::sig::Vector &right);
		
		void set_redundant_task(const unsigned int &num, const double &scalar);
		
	private:
		// Variables
		bool leftControl = false;
		bool rightControl = false;
		double startTime;					// Used to regulate the control loop
		double alpha = 1;					// Gain on the redundant task for GPM
		int controlSpace = 100;					// 1 = Joint, 2 = Cartesian
		int nullSpaceTask = 1;					// 0 = Nothing, 1 = Manipulability, 2 = Stiffness
		yarp::sig::Vector q;					// Joint position vector
						
		// Objects
		ArmCtrl leftArm, rightArm;				// Handles kinematics for the arms
		JointCtrl torso;					// Handles joint communication and control for torso
		
		// Control thread functions
		bool threadInit();					// Executed after start() and before run()
		void run();						// Main control loop
		void threadRelease();					// Executed after stop is called
			
		// Joint Limit Avoidance Functions
		yarp::sig::Matrix get_joint_weighting();		// For joint limit avoidance in RMRC
		void scale_vector(yarp::sig::Vector &input, const yarp::sig::Vector ref); // Scale a vector based on speed limits
		
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
		yarp::sig::Vector q(10);						
		yarp::sig::Vector temp = this->torso.get_joint_positions();		// Get the joint positions from the torso
		for(int i = 0; i < 3; i++) q[i] = temp[2-i];				// Assign torso joints (in reverse order)
		
		q.setSubvector(3, this->leftArm.get_joint_positions());			// Get the joint positions from the left arm
		this->leftArm.set_joint_angles(q);					// Assign them to the iCubArm object in this class
		
		q.setSubvector(3, this->rightArm.get_joint_positions());		// Overwrite with the right arm joints
		this->rightArm.set_joint_angles(q);					// Assign them to the iCubArm object in this class
	}
	else	close();								// There was a problem, so shut down the robot
	
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
/******************** Move the hands by the given amount ********************/
void DualArmCtrl::translate(const yarp::sig::Vector &left, const yarp::sig::Vector &right)
{
	yarp::sig::Matrix TL, TR;						// Homogeneous transform

	// Offset the left hand by the given amount
	if(left.size() == 3)
	{
		TL = this->leftArm.get_pose();					// Get the pose of the left hand
		for(int i = 0; i < 3; i++) TL[i][3] += left[i];			// Add the translation
	}

	// Offset the right hand by the given amount
	if(right.size() == 3)
	{
		TR = this->rightArm.get_pose();					// Get the pose of the right hand
		for(int i = 0; i < 3; i++) TR[i][3] += right[i];		// Add the translation
	}
	
	// Send the control
	if(TL.rows() != 0 || TR.rows() != 0) move_to_pose(TL, TR);
	else
	{
		yError() << "DualArmCtrl::translate() : Incorrect dimensions for the input arguments!"
			<< " The left vector has" << left.size() << "elements and the right vector has"
			<< right.size() << "elements.";
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
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;		// Update the time in the control loop
	
	// Solve the relevant control
	switch(this->controlSpace)
	{
		case 1: // Joint control
		{
			this->torso.track_joint_trajectory(elapsedTime);
			if(this->leftControl)	this->leftArm.track_joint_trajectory(elapsedTime);
			if(this->rightControl)	this->rightArm.track_joint_trajectory(elapsedTime);
			break;
		}
		case 2: // Cartesian control
		{
		
			// Variables used in this scope
			yarp::sig::Matrix J(12,17);					// Combined Jacobian for both arms
			yarp::sig::Matrix JLeft, JRight;				// Sub matrices for left, right arm	
			yarp::sig::Vector xdot(12);					// Combined task Jacobian
			
			if(this->leftControl)						// If left arm control is active...
			{
				xdot.setSubvector(0, this->leftArm.get_cartesian_control(elapsedTime));
				JLeft = this->leftArm.get_jacobian();
				J.setSubmatrix(JLeft, 0, 0);
			}
			
			if(this->rightControl)						// If right arm control is active...
			{
				xdot.setSubvector(6, this->rightArm.get_cartesian_control(elapsedTime));
				JRight = this->rightArm.get_jacobian();
				J.setSubmatrix(JRight.submatrix(0,5,0,2),6,0);
				J.setSubmatrix(JRight.submatrix(0,5,3,9),6,10);
			}
			
			// Solve the differential inverse kinematics
			yarp::sig::Matrix W = get_joint_weighting();			// NOTE: ALREADY THE INVERSE
			yarp::sig::Matrix invWJt = W*J.transposed();			// Makes calcs a little easier
			yarp::sig::Matrix invJ = invWJt*get_inverse(J*invWJt);		// W^-1*J'*(J*W^-1*J')^-1
			yarp::sig::Vector qdot_R = invJ*xdot;				// Range space task
			scale_vector(qdot_R, qdot_R);					// Ensure kinematic feasibility
		
			// Compute the null space motion
			yarp::sig::Vector qdot_N(17);					// Null space task

			if(this->nullSpaceTask != 0)
			{
				yarp::sig::Vector gradL(10), gradR(10);			// Left and right hand gradient vectors
				switch(this->nullSpaceTask)
				{
					case 1:
					{
						gradL = get_manip_grad("left", JLeft);
						gradR = get_manip_grad("right", JRight);
						break;
					}
					case 2:
					{
						yarp::sig::Vector direction({0,0,1,0,0,0});
						gradL = get_force_grad("left", JLeft, direction);
						gradR = get_force_grad("right", JRight, direction);
						break;
					}
					default:
					{
						gradL.zero();
						gradR.zero();
					}
				}
				
				yarp::sig::Vector qdot_d(17);
				for(int i = 0; i < 7; i++)
				{
					if(i < 3)	qdot_d[i]    = (gradL[i] + gradR[i])/2; // Mean of the two torso components
							qdot_d[i+3]  = gradL[i+3];	// Left arm
							qdot_d[i+10] = gradR[i+3];	// Right arm
				}
				qdot_d *= this->alpha;					// Scale it
				
				// Project the desired task on to the null space
				yarp::sig::Matrix I(17,17); I.eye();			// Identity matrix
				qdot_N = (I - invJ*J)*qdot_d;				// Null space projection
			}
		
			yarp::sig::Vector qdot = qdot_R + qdot_N;			// Combine the range and null space vectors
			scale_vector(qdot_N, qdot);					// Ensure feasibility of the null space task
					
			// Send the commands to the motors
			this->torso.move_at_speed(yarp::sig::Vector({qdot[2], qdot[1], qdot[0]})); // IN REVERSE ORDER
			this->leftArm.move_at_speed(qdot.subVector(3,9));
			this->rightArm.move_at_speed(qdot.subVector(10,16));

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
yarp::sig::Matrix DualArmCtrl::get_joint_weighting()
{
	// Note to self: It's easier to compute the penalty function in
	// the MultiJointController class and send the result here.
	
	yarp::sig::Matrix W(17,17);							// Value to be returned
	
	for(int i = 0; i < 3; i++)	W[i][i] = this->torso.get_joint_weight(2-i);	// Torso joints are backwards in the kinematic chain
	for(int i = 0; i < 7; i++)
	{
					W[i+3][i+3] = this->leftArm.get_joint_weight(i);
					W[i+10][i+10] = this->rightArm.get_joint_weight(i);	
	}
	return W;
}

/******************** Scale a task vector to remain within joint limits ********************/
void DualArmCtrl::scale_vector(yarp::sig::Vector &input, const yarp::sig::Vector ref)
{
	double s = 1.0;										// Scaling value
	double vMin, vMax;									// Maximum and minimum speed limits for a joint
	for(int i = 0; i < 17; i++)
	{
		// Get the speed limits for the current joint
		if(i < 3)		this->torso.get_speed_limits(2-i, vMin, vMax);		// Joints 0 - 2   : Torso (N.B. in reverse order!)
		else if(i < 10)		this->leftArm.get_speed_limits(i-3, vMin, vMax);	// Joints 3 - 9   : Left Arm
		else if(i < 17)		this->rightArm.get_speed_limits(i-10, vMin, vMax);	// Joints 10 - 16 : Right Arm
		
		if(ref[i] <= vMin && vMin/ref[i] < s) s = 0.99*vMin/ref[i];			// Scale if below limits and largest violation so far
		if(ref[i] >= vMax && vMax/ref[i] < s) s = 0.99*vMax/ref[i];			// Scale if above limits and largest violation so far
	}
	input *= s;										// Return the scaled vector
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
	yarp::sig::Matrix dJdq;
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
		if(which == "left")		dJdq = this->leftArm.get_partial_jacobian(i);
		else if(which == "right")	dJdq = this->rightArm.get_partial_jacobian(i);

		grad[i] = mu*trace(dJdq*Jacobian.transposed()*invJJt);
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

	yarp::sig::Matrix dJdq;
	yarp::sig::Vector Jtw = Jacobian.transposed()*wrench;				// Makes calcs a little faster
	yarp::sig::Vector grad(10);							// Value to be returned
	
	for(int i = 0; i < 10; i++)
	{
		if(which == "left")		dJdq = this->leftArm.get_partial_jacobian(i);
		else if(which == "right")	dJdq = this->rightArm.get_partial_jacobian(i);
		
		grad[i] = dot_product(dJdq.transposed()*wrench, Jtw);
	}
	return grad;
}

