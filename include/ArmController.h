#include <CartesianTrajectory.h>							// Custom trajectory object
#include <iCub/iKin/iKinFwd.h>							// iCub::iKin::iCubArm object
#include <MultiJointController.h>							// Custom low-level interface with robot

class ArmController : public MultiJointController
{
	public:
		ArmController() {};							// Empty constructor
		
		void configure(const std::string &local_port_name,
				const std::string &remote_port_name,
				const std::string &_name);
				
		void set_cartesian_trajectory(const yarp::sig::Matrix &desired,	// Set new Cartesian trajectory
						const double &time);
						
		yarp::sig::Vector cartesian_control(const double &time);		// Get the end-effector velolcity to track the trajectory
		
		
		yarp::sig::Matrix get_jacobian() {return this->arm.GeoJacobian();}
		
		void set_angles(yarp::sig::Vector &input) {this->arm.setAng(input*M_PI/180);}		
		
	private:
		CartesianTrajectory trajectory;					// Self explanatory
		
		iCub::iKin::iCubArm arm;						// iCubArm object
		
		yarp::sig::Matrix pos;							// Desired pose for the hand
		
		yarp::sig::Vector vel, acc, err, angleAxis;				// Desired velocity, acceleration, pose error

};											// Semicolon needed after a class declaration

/******************** Set new Cartesian trajectory to track ********************/
void ArmController::set_cartesian_trajectory(const yarp::sig::Matrix &desired,
						const double &time)
{
	// NOTE: Ensure joint state is updated properly BEFORE calling this function!
	
	if(desired.rows() != 4 || desired.rows() != 4)
	{
		yError("ArmController::set_cartesian_trajectory() : Expected a 4x4 homeogenous transform as an input");
	}
	else
	{
		this->trajectory = CartesianTrajectory(this->arm.getH(),desired, 0, time);
	}
}

/******************** Solve the Cartesian control to follow the trajectory ********************/
yarp::sig::Vector ArmController::cartesian_control(const double &time)
{
	// NOTE TO SELF: If there's a problem with orientation feedback, it's probably here!
	
	yarp::sig::Matrix T = this->arm.getH();						// Get the actual end-effector pose
	
	this->trajectory.get_state(this->pos, this->vel, this->acc, time);		// Get desired state for current time
	
	yarp::sig::Vector angleAxis = yarp::math::dcm2axis(pos*yarp::math::SE3inv(T)); // Compute the orientation error
	
	for(int i = 0; i < 3; i++)
	{
		this->err[i]	= this->pos[i][3] - T[i][3];				// Position error
		this->err[i+3]	= this->angleAxis[0]*this->angleAxis[i];		// Angle*Axis
	}

	return this->vel + 2*this->err;						// Return feedforward + feedback control
}

/******************** Configure the left arm ********************/
void ArmController::configure(const std::string &local_port_name,
				const std::string &remote_port_name,
				const std::string &_name)
{	
	// Resize vectors & arrays
	this->pos.resize(4,4);
	this->vel.resize(6);
	this->acc.resize(6);
	this->err.resize(6);
	this->angleAxis.resize(4);

	// Set the iCubArm object
	this->arm = iCub::iKin::iCubArm(_name+"_v2");
	this->arm.setAllConstraints(false);						// I don't know what this does
	for(int i = 0; i < 3; i++) this->arm.releaseLink(i);				// Release all the torso joints for us
	yInfo() << "This arm is the " << this->arm.getType() << " arm.";		// Inform the user
	
	configure_drivers(local_port_name, remote_port_name, _name+" arm", 7);	// We only need 7 joints in the arm
}
