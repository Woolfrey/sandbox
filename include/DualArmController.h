#include <iostream>							// For debugging
#include <ArmController.h>						// Handles forward kinematics
#include <yarp/os/PeriodicThread.h>					// Control loop

/*
	THINGS TO FIX:
		- No switch to deactivate torso control - could be a problem
		  with null inputs.
*/

class DualArmController : public yarp::os::PeriodicThread
{
	public:
		DualArmController();					// Constructor
		
		void close();						// Close the device drivers
		
		void update_state();					// Update the current joint state
		
		void move_to_position(yarp::sig::Vector &left,
					yarp::sig::Vector &right,
					yarp::sig::Vector &mid);
					
		void move_lateral(const double &distance);
		void move_vertical(const double &distance);
		void move_in_out(const double &distance);
		void move_horizontal(const double &distance);
		
		yarp::sig::Matrix get_joint_weighting();
		
		yarp::sig::Matrix get_pseudoinverse(const yarp::sig::Matrix &J,
							const yarp::sig::Matrix &W);
							
		void print_pose(const std::string &which);
		
		bool threadInit();					// Executed after start() and before run()
		void run();						// Main control loop
		void threadRelease();					// Executed after stop is called
			
	private:
		bool leftControl = false;
		bool rightControl = false;
		
		double elapsedTime, startTime;				// Used to regulate the control loop
		
		int controlSpace = 100;					// 1 = Joint, 2 = Cartesian
		
		ArmController leftArm, rightArm;			// Handles kinematics for the arms
		MultiJointController torso;				// Handles joint communication and control for torso
		
		yarp::sig::Vector xdot, qdot;
		yarp::sig::Matrix J, j;
		
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
	// Resize vectors, matrices
	this->J.resize(12,17);
	this->xdot.resize(12);
	this->qdot.resize(17);
	
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
	this->torso.read_encoders();
	this->leftArm.read_encoders();
	this->rightArm.read_encoders();
	
	// Put the values in to the iCub::iKin::iCubArm object
	yarp::sig::Vector q(10);						// Need to put this in to the iCubArm object
	
	yarp::sig::Vector temp = this->torso.get_joint_positions();		// Get the joint positions for the torso
	for(int i = 0; i < 3; i++) q[i] = temp[2-i];				// Torso joints are in reverse order
	
	q.setSubvector(3, this->leftArm.get_joint_positions());			// Append the left arm joints
	this->leftArm.set_joint_angles(q);					// Assign them to the iCubArm object
	
//	yInfo() << "Here is the joint position vector for the left-arm kinematic chain:";
//	std::cout << temp.toString() << std::endl;
	
	q.setSubvector(3, this->rightArm.get_joint_positions());			// Overwrite with the right arm joints
	this->rightArm.set_joint_angles(q);					// Assign them to the iCubArm object
	
//	yInfo() << "Here is the joint position vector for the right-arm kinematic chain:";
//	std::cout << temp.toString() << std::endl;
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
		
//		yInfo() << "Here is the left arm input in degrees:";
//		std::cout << left.toString() << std::endl;	
		
//		yInfo() << "And here it is in radians:";
//		std::cout << left.toString() << std::endl;
		
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
	this->leftArm.set_cartesian_trajectory(H, 3.0);				// Set the internal trajectory object
	this->leftControl = true;
	
	// Compute the right arm control
	H = this->rightArm.get_pose();
	H[1][3] -= distance;							// OPPOSITE GLOBAL FRAME
	this->rightArm.set_cartesian_trajectory(H, 3.0);
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
	this->leftArm.set_cartesian_trajectory(H, 3.0);				// Set the internal trajectory object
	this->leftControl = true;
	
	// Compute the right arm control
	H = this->rightArm.get_pose();
	H[2][3] += distance;							// Offset the z position
	this->rightArm.set_cartesian_trajectory(H, 3.0);
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
	this->leftArm.set_cartesian_trajectory(H, 3.0);				// Set the internal trajectory object
	this->leftControl = true;
	
	// Compute the right arm control
	H = this->rightArm.get_pose();
	H[1][3] -= distance;
	this->rightArm.set_cartesian_trajectory(H, 3.0);
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
	this->leftArm.set_cartesian_trajectory(H, 3.0);				// Set the internal trajectory object
	this->leftControl = true;
	
	// Compute the right arm control
	H = this->rightArm.get_pose();
	H[0][3] -= distance;
	this->rightArm.set_cartesian_trajectory(H, 3.0);
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
			
			if(this->leftControl)						// Left arm control active
			{	
				// Solve the Cartesian control part
				this->xdot.setSubvector(0,this->leftArm.get_cartesian_control(this->elapsedTime));
				
				// Construct the Jacobian
				this->J.setSubmatrix(this->leftArm.get_jacobian(),0,0);	// Effect on left hand
			}
			
			if(this->rightControl)						// Right arm control active
			{
				// Solve the Cartesian control part
				this->xdot.setSubvector(6,this->rightArm.get_cartesian_control(this->elapsedTime));
				
				// Construct the Jacobian
				this->j = this->rightArm.get_jacobian();		// Get the Jacobian for the current state
				
				this->J.setSubmatrix(this->j.submatrix(0,5,0,2),6,0);	// Assign torso effect for right hand
				this->J.setSubmatrix(this->j.submatrix(0,5,3,9),6,10);	// Assign arm effect for the right hand
			}
			
//			yInfo("Here is the Jacobian:");
//			std::cout << this->J.toString() << std::endl;
			
			// Solve differential inverse kinematics
			yarp::sig::Matrix W = get_joint_weighting();			// NOTE: This is already the inverse!
			yarp::sig::Matrix invJ = get_pseudoinverse(J,W);		// Get the weighted pseudoinverse Jacobian
			//yarp::sig::Matrix I(17,17);
			//I.zero();
			
			yarp::sig::Vector qdot = invJ*xdot;// + (I - invJ*J)*rdot;
			
//			yInfo("Here is the desired Cartesian velocity:");
//			std::cout << this->xdot.toString() << std::endl;

			// Send the commands to the motors
			this->torso.move_at_speed(yarp::sig::Vector({qdot[2], qdot[1], qdot[0]})); // IN REVERSE ORDER
			this->leftArm.move_at_speed(qdot.subVector(3,9));
			this->rightArm.move_at_speed(qdot.subVector(10,16));
/*			
			yarp::sig::Vector blah(17);
			blah.setSubvector(0,this->leftArm.get_joint_velocities());
			blah.setSubvector(7,this->rightArm.get_joint_velocities());
			yarp::sig::Vector temp = this->torso.get_joint_velocities();
			for(int i = 0; i < 3; i++) blah[14+i] = temp[2-i];
			
			yInfo("Here is the actual Cartesian velocity:");
			std::cout << (this->J*blah).toString() << std::endl;
*/
			
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
yarp::sig::Matrix DualArmController::get_joint_weighting()
{
	// Note to self: It's easier to compute the penalty function in
	// the MultiJointController class and send the result here.

	yarp::sig::Matrix W(17,17);						// Value to be returned
	W.eye();
/*
	for(int i = 0; i < 7; i++)
	{
		W[i][i]		= this->leftArm.get_joint_weight(i);
		W[i+7][i+7]	= this->rightArm.get_joint_weight(i);
	}
	
	for(int i = 0; i < 3; i++) W[i+14][i+14] = this->torso.get_joint_weight(i);
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
		yarp::sig::Matrix A = J*invWJt;						// We want to invert this matrix
		
		yarp::sig::Matrix U(A.rows(), A.rows()), V(A.cols(), A.cols()); 	// Orthogonal matrices
		yarp::sig::Vector s(A.rows());						// List of singular values
		
		yarp::math::SVD(A,U,s,V);						// Get the singular value decomposition

		yarp::sig::Matrix invA(A.rows(), A.cols());
		invA.zero();
		for(int i = 0; i < A.rows(); i++)
		{
			for(int j = 0; j < A.cols(); j++)
			{
				if(s(j) == 0);	//	invA(i,J) += 0			// Ignore singular directions
				else if(s(j) < 1e-05)	invA(i,j) += V(i,j)*1e05;	// Damp near-singular directions
				else			invA(i,j) += V(i,j)/s(j);	// Proper inversion of the first half
			}
		}
		
		return invWJt*invA*U.transposed();					// Return the complete inversion
	}
}


/*

class DualArmController : virtual public yarp::os::PeriodicThread
{
	public:
		DualArmController();						// Constructor
		
		void update_state();						// Update state for all objects
		void move_to_position(yarp::sig::Vector &left,			// Move the joints to a desired configuration
					yarp::sig::Vector &right,
					yarp::sig::Vector &torso);
			
		bool threadInit();						// Executed after start() and before run()
		void run();							// Main control loop
		void threadRelease();						// Executed after stop is called
		void close();
		void move_lateral(const double &distance);			// Move hands in the y-direction
		void move_vertical(const double &distance);			// Move the hands in the z-direction
		
		void blah();
		
		yarp::sig::Vector get_manipulability_gradient();			
		
		yarp::sig::Vector get_positions(const std::string &which);	
	
	private:
		// These are used to regulate the arm control
		int controlMode;						// 1 = Joint, 2 = Cartesian, 3 = Hybrid
		bool controlSwitch[2];						// Switch for activating control of the arms
		double startTime, elapsedTime;					// For timing in the control
		
		// These handle arm and joint kinematics
		ArmController leftArm, rightArm;				// Handles Cartesian stuff
		MultiJointController torso;					// Low level interface for the torso joints
		
		// These are used in the Cartesian control
		yarp::sig::Matrix J;						// Jacobian for both arms
		yarp::sig::Matrix j;						// Jacobian for a single arm
		yarp::sig::Vector xdot;						// Task velocities
		
		// Functions
		yarp::sig::Matrix get_joint_weighting();			// Weight the joint motions
		yarp::sig::Matrix get_pseudoinverse(const yarp::sig::Matrix &J,
						   const yarp::sig::Matrix &W);		

};										// Semicolon needed after a class declaration

/******************** Print outputs to check RMRC ********************
void DualArmController::blah()
{
	this->J.zero();								// Clear the Jacobian
	
	this->elapsedTime = 0.5;

	// Solve the Cartesian control part
	this->xdot.setSubvector(0,this->leftArm.get_cartesian_control(this->elapsedTime));
	//this->xdot.setSubvector(6,this->rightArm.get_cartesian_control(this->elapsedTime));
	
	yInfo("Here is the task velocity vector:");
	std::cout << xdot.toString() << std::endl;
	
	// Construct the Jacobian
	this->j = this->leftArm.get_jacobian();					// Get the Jacobian for the current state
	this->J.setSubmatrix(this->j.submatrix(0,5,0,6),0,0);			// Allocate Jacobian for arm control
	this->J.setSubmatrix(this->j.submatrix(0,5,7,9),0,14);			// Allocate the Jacobian for the torso
	
	//this->j = this->leftArm.get_jacobian();					// Get the Jacobian for the current state
	//this->J.setSubmatrix(this->j.submatrix(0,5,0,6),6,7); 			// Allocate Jacobian for arm control
	//this->J.setSubmatrix(this->j.submatrix(0,5,7,9),6,14); 			// Allocate the Jacobian for the torso
	
	yInfo("Here is the Jacobian matrix:");
	std::cout << this->J.toString() << std::endl;
		
	// Solve differential inverse kinematics
	yarp::sig::Matrix W = get_joint_weighting();				// NOTE: This is already the inverse!
	
	yInfo("Here is the joint weighting:");
	std::cout << W.toString() << std::endl;
	
	yarp::sig::Matrix invJ = get_pseudoinverse(J,W);			// Get the weighted pseudoinverse Jacobian

	yInfo("Here is the pseudoinverse Jacobian:");
	std::cout << invJ.toString() << std::endl;
	
	yInfo("Jacobian by the inverse:");
	std::cout << (J*invJ).toString() << std::endl;
	
	yarp::sig::Vector qdot = invJ*xdot;// + (I - invJ*J)*rdot;
	qdot *= 180/M_PI;							// Convert to deg/s
	
	yInfo("Joint velocity vector:");
	std::cout << qdot.toString() << std::endl;
	
	yarp::sig::Matrix I(17,17);
	I.eye();
	yarp::sig::Matrix N = I - invJ*J;
	
	yInfo("Here is the null space projection matrix:");
	std::cout << N.toString() << std::endl;
	
	yInfo("Jacobian by the null space:");
	std::cout << (J*N).toString() << std::endl;
	
	//yInfo("Joint velocity:");
	//std::cout << qdot.toString() << std::endl;
}

/******************** Move the joints to desired configuration ********************
void DualArmController::move_to_position(yarp::sig::Vector &left, yarp::sig::Vector &right, yarp::sig::Vector &mid)
{
	if(isRunning()) stop();						// Stop any threads that are running

	if(left.size() != 0)	this->controlSwitch[0] = true;
	else			this->controlSwitch[0] = false;
	
	if(right.size() != 0)	this->controlSwitch[1] = true;
	else			this->controlSwitch[1] = false;
	
	
	if(!this->controlSwitch[0] && !this->controlSwitch[1] && mid.size() == 0)
	{
		yError("DualArmController::move_to_position() : Both arguments have null inputs!");
	}
	else
	{
		this->controlMode = 1;	// Use to trigger joint control mode
		if(this->controlSwitch[0]) this->leftArm.set_joint_trajectory(left);
		if(this->controlSwitch[1]) this->rightArm.set_joint_trajectory(right);
		this->torso.set_joint_trajectory(mid);
	}
	
	start(); // Go immediately to threadInit() 
}

/******************** Cartesian Control Functions ********************
void DualArmController::move_lateral(const double &distance)
{
	if(isRunning()) stop();							// Stop any control threads
	
	this->controlMode = 2;							// Switch case for Cartesian control
	
	yarp::sig::Matrix H = this->leftArm.get_pose();				// Get the pose of the left hand
	
	H[1][3] += distance;							// Offset the y-position
	
	this->leftArm.set_cartesian_trajectory(H, 3.0);				// Set the internal trajectory object
	this->controlSwitch[0] = true;						// Activate left arm control
	this->controlSwitch[1] = false;						// Ensure right is turned off
	
	start(); // Go immediately to initThread();
}
void DualArmController::move_vertical(const double &distance)
{
	if(isRunning()) stop();							// Stop any control threads
	
	this->controlMode = 2;							// Switch case for Cartesian control
	
	yarp::sig::Matrix H = this->leftArm.get_pose();				// Get the pose of the left hand
	
	H[2][3] += distance;							// Offset the y-position
	
	this->leftArm.set_cartesian_trajectory(H, 3.0);				// Set the internal trajectory object
	this->controlSwitch[0] = true;						// Activate left arm control
	this->controlSwitch[1] = false;						// Ensure right is turned off
	
	start(); // Go immediately to initThread();
}

/******************** Executed just before run() ********************
bool DualArmController::threadInit()
{
	this->startTime = yarp::os::Time::now();				// Start the timer for the control loop
	return true;
	// Now go immediately to run()
}

/******************** MAIN CONTROL LOOP ********************
void DualArmController::run()
{
	update_state();								// Update the joint state information
	
	this->elapsedTime = yarp::os::Time::now() - this->startTime;		// Update the time in the control loop
	
	// Solve the relevant control
	switch(this->controlMode)
	{
		case 1: // Joint control
		{
			// this->torso.joint_control(this->elapsedTime);
			if(this->controlSwitch[0]) this->leftArm.joint_control(this->elapsedTime);
			if(this->controlSwitch[1]) this->rightArm.joint_control(this->elapsedTime);
			this->torso.joint_control(this->elapsedTime);
			
			break;
		}
		case 2: // Cartesian control
		{			
			this->J.zero();							// Clear the Jacobian
			this->xdot.zero();						// Clear the task vector
			
			if(this->controlSwitch[0])					// Left arm control active
			{	
				// Solve the Cartesian control part
				this->xdot.setSubvector(0,this->leftArm.get_cartesian_control(this->elapsedTime));
				
				// Construct the Jacobian
				this->j = this->leftArm.get_jacobian();			// Get the Jacobian for the current state
				this->J.setSubmatrix(this->j.submatrix(0,5,0,6),0,0);	// Allocate Jacobian for arm control
				this->J.setSubmatrix(this->j.submatrix(0,5,7,9),0,14);	// Allocate the Jacobian for the torso
			}
			
			if(this->controlSwitch[1])					// Right arm control active
			{
				// Solve the Cartesian control part
				this->xdot.setSubvector(6,this->rightArm.get_cartesian_control(this->elapsedTime));
				
				// Construct the Jacobian
				this->j = this->leftArm.get_jacobian();			// Get the Jacobian for the current state
				this->J.setSubmatrix(this->j.submatrix(0,5,0,6),6,7);	// Allocate Jacobian for arm control
				this->J.setSubmatrix(this->j.submatrix(0,5,7,9),6,14);	// Allocate the Jacobian for the torso
			}
						
			// Solve differential inverse kinematics
			yarp::sig::Matrix W = get_joint_weighting();			// NOTE: This is already the inverse!
			yarp::sig::Matrix invJ = get_pseudoinverse(J,W);		// Get the weighted pseudoinverse Jacobian
			//yarp::sig::Matrix I(17,17);
			//I.zero();
			
			yarp::sig::Vector qdot = invJ*xdot;// + (I - invJ*J)*rdot;
			qdot *= 180/M_PI;						// Convert to deg/s
			
			// Send commands to the motors
			this->leftArm.move_at_speed(qdot.subVector(0,6));
			this->rightArm.move_at_speed(qdot.subVector(7,13));
			this->torso.move_at_speed(yarp::sig::Vector({qdot[16], qdot[15], qdot[14]})); // NOTE: TORSO JOINTS ARE IN REVERSE
			
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

/******************** Executed after stop() is called ********************
void DualArmController::threadRelease()
{
	this->torso.move_at_speed(yarp::sig::Vector({0,0,0}));
	this->leftArm.move_at_speed(yarp::sig::Vector({0,0,0,0,0,0,0}));
	this->rightArm.move_at_speed(yarp::sig::Vector({0,0,0,0,0,0,0}));
}


/******************** Get the joint weighting matrix ********************
yarp::sig::Matrix DualArmController::get_joint_weighting()
{
	// Note to self: It's easier to compute the penalty function in
	// the MultiJointController class and send the result here.

	yarp::sig::Matrix W(17,17);						// Value to be returned
	W.eye();

	for(int i = 0; i < 7; i++)
	{
		W[i][i]		= this->leftArm.get_joint_weight(i);
		W[i+7][i+7]	= this->rightArm.get_joint_weight(i);
	}
	
	for(int i = 0; i < 3; i++) W[i+14][i+14] = this->torso.get_joint_weight(i);
	
	return W;
}

/******************** Get the weighted pseudoinverse Jacobian ********************
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
		yarp::sig::Matrix A = J*invWJt;						// We want to invert this matrix
		
		yarp::sig::Matrix U(A.rows(), A.rows()), V(A.cols(), A.cols()); 	// Orthogonal matrices
		yarp::sig::Vector s(A.rows());						// List of singular values
		
		yarp::math::SVD(A,U,s,V);						// Get the singular value decomposition

		yarp::sig::Matrix invA(A.rows(), A.cols());
		invA.zero();
		for(int i = 0; i < A.rows(); i++)
		{
			for(int j = 0; j < A.cols(); j++)
			{
				if(s(j) == 0);	//	invA(i,J) += 0			// Ignore singular directions
				else if(s(j) < 1e-05)	invA(i,j) += V(i,j)*1e05;	// Damp near-singular directions
				else			invA(i,j) += V(i,j)/s(j);	// Proper inversion of the first half
			}
		}
		
		return invWJt*invA*U.transposed();					// Return the complete inversion
	}
}

/******************** Get and set new joint state information ********************
void DualArmController::update_state()
{
	this->torso.get_encoder_values();				// Read encoders on torso joints
	this->leftArm.get_encoder_values();
	this->rightArm.get_encoder_values();
	
	yarp::sig::Vector dof(10);					// We need to put this in to the iCubArm objects
	
	for(int i = 0; i < 3; i++) dof[i] = torso.q[2-i];		// Torso joint order is opposite to the arms
	
	for(int i = 0; i < 7; i++) dof[i+3] = this->leftArm.q[i];	// Assign the left arm joint angles to the stack
	this->leftArm.set_angles(dof);
	
	for(int i = 0; i < 7; i++) dof[i+3] = this->rightArm.q[i];	// Assign the right arm joint angles to the stack
	this->rightArm.set_angles(dof);
}

/******************** Constructor ********************
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
	update_state();
	
	for(int i = 0; i < 2; i++) this->controlSwitch[i] = false;		// Default starting value
}

/******************** Close the device drivers ********************
void DualArmController::close()
{
	stop();									// Stop control threads
	this->leftArm.close();							// Close device drivers
	this->rightArm.close();
	this->torso.close();
}

/******************** Get the gradient of the manipulability measure ********************
yarp::sig::Vector DualArmController::get_manipulability_gradient()
{
	yarp::sig::Vector grad(17);						// Value to be returned

	return grad;
}

/******************** Get the joint positions for a single arm ********************
yarp::sig::Vector DualArmController::get_positions(const std::string &which)
{
	if(which.compare("left"))	return this->leftArm.q;
	else if(which.compare("right"))	return this->rightArm.q;
	else if(which.compare("torso"))	return this->torso.q;
	else
	{
		yError("DualArmController::get_positions() : Expected string input as 'left' or 'right'.");
		return yarp::sig::Vector({0,0,0,0,0,0,0});
	}
}
*/
