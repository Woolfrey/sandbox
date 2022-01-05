#include <iostream>									// For debugging
#include <ArmCtrl.h>
#include <yarp/os/PeriodicThread.h>

class StiffnessCtrl : public virtual yarp::os::PeriodicThread
{
	public:
		StiffnessCtrl(const double &freq);					// Constructor
		void close();
		bool update_state();
		void move_to_position(yarp::sig::Vector &left, yarp::sig::Vector &right, yarp::sig::Vector &mid);
		void move_to_pose(const yarp::sig::Matrix &pose);			// Move the right hand
		void set_direction(const std::string &dir);
		bool set_joint_limits(const int &i,
				const double &lower,
				const double &upper,
				const std::string &which); // Set new joint limits
	
	private:
		ArmCtrl leftArm, rightArm;
		double startTime;							// Helps regulate control loop
		int controlSpace = 1;
		JointCtrl torso;
		yarp::sig::Vector direction;
		
		// Functions
		double dot_product(const yarp::sig::Vector &a, const yarp::sig::Vector &b);
		yarp::sig::Matrix get_inverse(const yarp::sig::Matrix &A);
		yarp::sig::Matrix get_joint_weighting();				// For joint limit avoidance
		yarp::sig::Vector get_force_grad(const std::string &which, const yarp::sig::Matrix &Jacobian, const yarp::sig::Vector &wrench);
		void scale_vector(yarp::sig::Vector &vec, const yarp::sig::Vector ref); // For joint limit avoidance
		
		// Control Loop Functions
		virtual bool threadInit();
		virtual void run();
		virtual void threadRelease();
};											// Semicolon needed after a class declaration

/******************** Constructor ********************/
StiffnessCtrl::StiffnessCtrl(const double &freq) : PeriodicThread(1/freq),
					    	   direction(yarp::sig::Vector({0,1,0,0,0,0}))
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
	
	update_state();	
}
/******************** Get and set new joint state information ********************/
bool StiffnessCtrl::update_state()
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
void StiffnessCtrl::close()
{
	stop();										// Stop any control threads first
	this->leftArm.close();
	this->rightArm.close();
	this->torso.close();
}

/******************** Move the joints to desired configuration ********************/
void StiffnessCtrl::move_to_position(yarp::sig::Vector &left, yarp::sig::Vector &right, yarp::sig::Vector &mid)
{
	if(isRunning()) stop();
	this->controlSpace = 1;						// Switch case for control
	
	// Generate new joint space trajectories internally
	this->torso.set_joint_trajectory(mid);
	this->leftArm.set_joint_trajectory(left);
	this->rightArm.set_joint_trajectory(right);
	
	start(); // Go immediately to threadInit() 
}

/******************** Move the right hand to the given target ********************/
void StiffnessCtrl::move_to_pose(const yarp::sig::Matrix &pose)			// Move the right hand
{	
	if(pose.rows() != 4 || pose.cols() != 4)
	{
		yError() << "StiffnessCtrl::move_right_hand() : Expected an SO3 matrix! Here's what you input:";
		yInfo() << pose.toString();
	}
	else
	{
		if(isRunning()) stop();						// Stop any threads that are running
		this->controlSpace = 2;
		this->rightArm.set_cartesian_trajectory(pose, 3.0);
		start(); // Go immediately to threadInit()	
	}
}

void StiffnessCtrl::set_direction(const std::string &dir)
{
	if(dir == "x") this->direction = yarp::sig::Vector({1,0,0,0,0,0});
	if(dir == "y") this->direction = yarp::sig::Vector({0,1,0,0,0,0});
	if(dir == "z") this->direction = yarp::sig::Vector({0,0,1,0,0,0});
}

bool StiffnessCtrl::set_joint_limits(const int &i, const double &lower, const double &upper, const std::string &which)
{
	if(which == "torso")
	{
		this->torso.set_joint_limits(i, lower, upper);
		return true;
	}
	else if(which == "left")
	{
		this->leftArm.set_joint_limits(i, lower, upper);
		return true;
	}
	else if(which == "right")
	{
		this->rightArm.set_joint_limits(i, lower, upper);
		return true;
	}
	else
	{
		yError("StiffnessCtrl::set_joint_limits() : Must specify 'torso', 'left', or 'right'");
		return false;
	}
}

/******************** Executed after start() and before run() ********************/
bool StiffnessCtrl::threadInit()
{
	this->startTime = yarp::os::Time::now();					// Start timer for the control loop
	return true;
}

/******************** MAIN CONTROL LOOP ********************/
void StiffnessCtrl::run()
{
	update_state();									// Update the joint state information
	
	double elapsedTime = yarp::os::Time::now() - this->startTime;			// Update the time in the control loop
	
	switch(this->controlSpace)
	{
		case 1:
		{
			this->torso.track_joint_trajectory(elapsedTime);
			this->leftArm.track_joint_trajectory(elapsedTime);
			this->rightArm.track_joint_trajectory(elapsedTime);
			break;
		}
		case 2:
		{
			// Get reduced Cartesian control
			yarp::sig::Vector xdotFull = this->rightArm.get_cartesian_control(elapsedTime);
			yarp::sig::Vector xdot(2);
			xdot[0] = xdotFull[1]; // y translation
			xdot[1] = xdotFull[2]; // z translation
			
			// Get a reduced Jacobian matrix
			yarp::sig::Matrix JFull = this->rightArm.get_jacobian();
			yarp::sig::Matrix J(2,10);
			for(int i = 0; i < 10; i++)
			{
				J[0][i] = JFull[1][i]; // y translation
				J[1][i] = JFull[2][i]; // z translation
			}

			// Solve velocity inverse kinematics
//			yarp::sig::Matrix W(10,10); W.eye();
			yarp::sig::Matrix W = get_joint_weighting();
			yarp::sig::Matrix invWJt = W*J.transposed();					// Makes calcs a little easier
			yarp::sig::Matrix invJ = invWJt*get_inverse(J*invWJt);				// Weighted pseudoinverse Jacobian
			yarp::sig::Vector qdot_R = invJ*xdot;						// Range space vector
			scale_vector(qdot_R, qdot_R);
			
			// Solve the redundant task
			yarp::sig::Vector qdot_d = -5.0*get_force_grad("right",JFull,direction);	// Gradient vector
			yarp::sig::Matrix I(10,10); I.eye();						// Identity matrix
			yarp::sig::Vector qdot_N = (I - invJ*J)*qdot_d;					// Null space projection
			
			yarp::sig::Vector qdot = qdot_R + qdot_N;
			scale_vector(qdot_N, qdot);
			qdot = qdot_R + qdot_N;
			
			// Send the commands to the motors
		//	this->torso.move_at_speed(yarp::sig::Vector({qdot[2], qdot[1], qdot[0]})); // IN REVERSE ORDER
			this->rightArm.move_at_speed(qdot.subVector(3,9));
			break;
		}
	}
//			Run this function repeatedly until stop() is called, then go to threadRelease();
}

/******************** Executed after stop() is called ********************/
void StiffnessCtrl::threadRelease()
{
	// Ensure the last command is zero velocity
	this->torso.move_at_speed(yarp::sig::Vector({0,0,0}));
	this->leftArm.move_at_speed(yarp::sig::Vector({0,0,0,0,0,0,0}));
	this->rightArm.move_at_speed(yarp::sig::Vector({0,0,0,0,0,0,0}));
}

void StiffnessCtrl::scale_vector(yarp::sig::Vector &vec, const yarp::sig::Vector ref)
{
	double s = 1.0;										// Scaling value
	double vMin, vMax;									// Maximum and minimum speed limits for a joint
	for(int i = 0; i < ref.size(); i++)
	{
		// Get the speed limits for the current joint
		if(i < 3)		this->torso.get_speed_limits(2-i, vMin, vMax);		// Joints 0 - 2   : Torso (N.B. in reverse order!)
		else if(i < 10)		this->rightArm.get_speed_limits(i-3, vMin, vMax);	// Joints 3 - 9   : Left Arm
		if(ref[i] <= vMin && vMin/ref[i] < s) s = 0.99*vMin/ref[i];			// Scale if below limits and largest violation so far
		if(ref[i] >= vMax && vMax/ref[i] < s) s = 0.99*vMax/ref[i];			// Scale if above limits and largest violation so far
	}
	vec *= s;	
}
/******************** Get the joint weighting matrix ********************/
yarp::sig::Matrix StiffnessCtrl::get_joint_weighting()
{
	// Note to self: It's easier to compute the penalty function in
	// the MultiJointController class and send the result here.
	
	yarp::sig::Matrix W(10,10);
	for(int i = 0; i < 7; i++)
	{
		if(i < 3)	W[i][i] = this->torso.get_joint_weight(2-i);			// Torso joints are backwards
				W[i+3][i+3] = this->rightArm.get_joint_weight(i);
	}
	return W;
}
/******************** Get the dot product between 2 vectors ********************/
double StiffnessCtrl::dot_product(const yarp::sig::Vector &a, const yarp::sig::Vector &b)
{
	if(a.size() != b.size())
	{
		yError("StiffnessCtrl::dot_product() : Vectors are not of equal length!");
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
/******************** Get the inverse of a matrix ********************/
yarp::sig::Matrix StiffnessCtrl::get_inverse(const yarp::sig::Matrix &A)
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
/******************** Get the gradient for static joint torques ********************/
yarp::sig::Vector StiffnessCtrl::get_force_grad(const std::string &which, const yarp::sig::Matrix &Jacobian, const yarp::sig::Vector &wrench)
{
	// Prepare the iKinChain object to compute the Hessian
	if(which == "left")		this->leftArm.prep_for_hessian();
	else if(which == "right")	this->rightArm.prep_for_hessian();
	else				yError("StiffnessCtrl::get_force_grad() : Expected 'left' or 'right' as an argument.");

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
