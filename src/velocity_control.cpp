#include <iostream>
#include <math.h>						// pow(x,n) function
#include <yarp/dev/ControlBoardInterfaces.h>			// Communicates with motor controllers on the robot?
#include <yarp/dev/PolyDriver.h>				// Device driver class
#include <yarp/os/LogStream.h>				// yarp::Info() and the like
#include <yarp/os/Network.h>					// Non so?
#include <yarp/os/Property.h>					// Don't know exactly what this does
#include <yarp/os/RpcServer.h>				// Allows communication between ports
#include <yarp/sig/Vector.h>					// For signal processing in YARP

/********************************* Simple Trajectory Class ***********************************/
class Quintic
{
	public:
	
	// Constructor
	Quintic(const yarp::sig::Vector &start_point, const yarp::sig::Vector &end_point, const double &start_time, const double &end_time)
	{
		this->p1 = start_point;
		this->p2 = end_point;
		this->t1 = start_time;
		this->t2 = end_time;
		this->m = this->p1.size();
		
		double dt = this->t2 - this->t1;
		this->a =   6*pow(dt,-5);
		this->b = -15*pow(dt,-4);
		this->c =  10*pow(dt,-3);
	}
	
	// Get the current desired position, velocity vectors	
	void getState(yarp::sig::Vector &pos, yarp::sig::Vector &vel, const double &t)
	{
		getScalars(t);
		for(int i = 0; i < this->m; i++)
		{
			pos[i] = (1 - this->s)*this->p1[i] + this->s*this->p2[i];
			vel[i] = this->sd*(this->p2[i] - this->p1[i]);
		}
	}
	
	void setPosition(const yarp::sig::Vector &start, const yarp::sig::Vector &end)
	{
		this->p1 = start;
		this->p2 = end;
	}
	
	void setTime(const double &start, const double &end)
	{
		this->t1 = start;
		this->t2 = end;
	}
	
	private:
	
	int m;								// Number of dimensions
	
	yarp::sig::Vector p1, p2, dp;					// Start point and end point
	
	double t1, t2;							// Start time and end time
	
	double a, b, c;						// Polynomial coefficients
	
	double s, sd;							// Interpolation scalars
	
	// Update the interpolating scalar values
	void getScalars(const double &t)
	{
		double dt;
		if(t < this->t1) 	dt = 0.0;			// Remain at the start
		else if(t < this->t2) 	dt = t - this->t1;		// Remain at the end
		else if(t >= t2) 	dt = this->t2 - this->t1;	// Somewhere inbetween
		
		this->s =    this->a*pow(dt,5) +   this->b*pow(dt,4) +   this->c*pow(dt,3);		// Position (normalised)
		this->sd = 5*this->a*pow(dt,4) + 4*this->b*pow(dt,3) + 3*this->c*pow(dt,2);		// Velocity
	}
};


/************************************** Forward Declarations ******************************************/

bool configureControl(yarp::dev::PolyDriver &driver, yarp::dev::IVelocityControl* &control, yarp::dev::IControlMode* &mode, yarp::sig::Vector &target);
bool configureDriver(yarp::dev::PolyDriver &driver, std::string &local, std::string &remote);
bool configureEncoder(yarp::dev::PolyDriver &driver, yarp::dev::IEncoders* &encoder, yarp::sig::Vector &position);
bool getEncoderValues(yarp::dev::IEncoders &enc, yarp::sig::Vector &storage);

void home(yarp::sig::Vector &left_setpoint,  yarp::dev::IEncoders &left_enc,  yarp::dev::IVelocityControl &left_ctrl,
 	  yarp::sig::Vector &right_setpoint, yarp::dev::IEncoders &right_enc, yarp::dev::IVelocityControl &right_ctrl);
 	  

/*********************************************** Main ************************************************/

int main(int argc, char *argv[])
{
	yarp::os::Network yarp;							// Need to establish this before using LogStream?
	
	// Configure the left arm
	std::string local = "/local/left";
	std::string remote = "/icubSim/left_arm";
	yarp::dev::PolyDriver left_driver;						// Create device driver object
	if(!configureDriver(left_driver, local, remote)) return 1;			// Configure and connect to iCub_SIM
	
	yarp::dev::IEncoders* left_encoder;						// Create an encoder object
	yarp::sig::Vector left_position;						// Create a vector to store encoder values
	if(!configureEncoder(left_driver, left_encoder, left_position)) return 1;	// Try and connect the encoder with the device driver
	
	yarp::dev::IControlMode *control_mode;					// Control mode object
	
	yarp::dev::IVelocityControl* left_control;					// Velocity controller for left arm
	yarp::sig::Vector left_target;						// Target positions for left arm
	if(!configureControl(left_driver, left_control, control_mode, left_target)) return 1; // Try and establish velocity control for left arm
	
	// Configure the right arm
	local = "/local/right";							// Name of the local port
	remote = "/icubSim/right_arm";						// Where we want to connect to
	yarp::dev::PolyDriver right_driver;						// Device driver object
	if(!configureDriver(right_driver, local, remote)) return 1;			// Configure and connect to iCub_SIM
	
	yarp::dev::IEncoders* right_encoder;						// Create encoder object
	yarp::sig::Vector right_position;						// Vector to store encoder values
	if(!configureEncoder(right_driver, right_encoder, right_position)) return 1;	// Try and connect encoder object with device driver
	
	yarp::dev::IVelocityControl* right_control;					// Velocity control object
	yarp::sig::Vector right_target;						// Target joint positions for right arm
	if(!configureControl(right_driver, right_control, control_mode, right_target)) return 1;
	
	// Get initial target values and move to home position
	left_target = left_position;							// Assign inital pose as home (to be overriden)
	right_target = right_position;
	
	// Move to home position
	yInfo("Moving to home position.");	
	home(left_target, *left_encoder, *left_control, right_target, *right_encoder, *right_control);


	// Configure communication
	yarp::os::RpcServer port;							// Create a port for sending and receiving information
	port.open("/command");								// Open the port with the name /command
	yarp::os::Bottle input;							// Store information from user input
	yarp::os::Bottle output;							// Store information to send to user
	std::string command;								// Response message, command from user
	
	left_driver.close();
	yDebug("So far so good.");

	return 0;									// No problems with main()
}

/*************************************** Other Functions ******************************************/

bool configureControl(yarp::dev::PolyDriver &driver, yarp::dev::IVelocityControl* &control, yarp::dev::IControlMode* &mode, yarp::sig::Vector &target)
{
	if(!driver.view(control))
	{
		yError("Unable to configure the controller.");
		driver.close();
		return 0;
	}
	else if(!driver.view(mode))
	{
		yError("Unable to configure the control mode.");
		driver.close();
		return 0;
	}
	else
	{
		int n;
		control->getAxes(&n);							// Get the number of joints
		target.resize(n);							// Resize target array accordingly
		
		for(int i = 0; i < n; i++)						// Cycle through all the joints
		{
			mode->setControlMode(i, VOCAB_CM_VELOCITY);			// Don't know what the second argument does
			control->setRefAcceleration(i, 50);				// Set acceleration (rad/s^2)
			control->velocityMove(i,0);					// Ensure inital velocity is zero
		}
		yInfo("Successfully configured velocity control.");
		return 1;
	}
}


bool configureDriver(yarp::dev::PolyDriver &driver, std::string &local, std::string &remote)
{
	yarp::os::Property options;
	options.put("device", "remote_controlboard");
	options.put("local", local);							// Local port names
	options.put("remote", remote);						// Where we want to connect to
	
	driver.open(options);
	
	if(!driver.isValid())					
	{
		yError("Device not available. Here are the known devices:");
		std::printf("%s", yarp::dev::Drivers::factory().toString().c_str());	
		return 0;								// Shut down
	}			
	else
	{
		yInfo("Successfully configured device driver.");
		return 1;
	}
}

bool configureEncoder(yarp::dev::PolyDriver &driver, yarp::dev::IEncoders* &encoder, yarp::sig::Vector &position)
{
	if(!driver.view(encoder))
	{
		yError("Could not configure encoder.");
		return 0;								// Unsuccessful
	}
	else
	{
		int n;
		encoder->getAxes(&n);							// Get the number of joints
		position.resize(n);							// Resize position vector accordingly
		if(!getEncoderValues(*encoder, position))				// Get initial encoder values and store them
		{
			driver.close();						// Close the driver
			return 0;
		}
		else
		{
			yInfo("Successfully configured encoder.");
			return 1;							// Success
		}
	}
}

bool getEncoderValues(yarp::dev::IEncoders &enc, yarp::sig::Vector &storage)
{
	for(int i = 0; i < 5; i++)							// Maximum five attempts
	{
		if(enc.getEncoders(storage.data())) return 1;				// Success!
		else yarp::os::Time::delay(0.01);					// Wait a little bit
	}
	yError("Could not get encoder values in 5 attempts.");			// Inform user
	return 0;									// Failure
}

void home(yarp::sig::Vector &left_setpoint,  yarp::dev::IEncoders &left_enc,  yarp::dev::IVelocityControl &left_ctrl,
 	  yarp::sig::Vector &right_setpoint, yarp::dev::IEncoders &right_enc, yarp::dev::IVelocityControl &right_ctrl)
{
	// Create vectors for the control
	yarp::sig::Vector left_values, right_values,					// Encoder values
			   left_pos, right_pos,					// Desired position
			   left_vel, right_vel,					// Desired velocity
			   left_ref, right_ref;					// Reference value for the control
	 
	// Resize the vectors accordingly
	int n;
	left_enc.getAxes(&n);
	left_values.resize(n);
	right_values.resize(n);
	left_pos.resize(n);
	right_pos.resize(n);
	left_vel.resize(n);
	right_vel.resize(n);
	left_ref.resize(n);
	right_ref.resize(n);
	
	// Get current encoder values and assign as inital values
	if(!getEncoderValues(left_enc, left_values)) 	 yError("You did a bad :(");
	if(!getEncoderValues(right_enc, right_values)) yError("You did a bad :(");
	left_setpoint = left_values;
	right_setpoint = right_values;
	
	// Overwrite the appropriate values (I should probably just hardcode every joint)
	for(int i = 0; i < 5; i++)
	{
		left_setpoint[i]  = 0.0;
		right_setpoint[i] = 0.0;
	}
	left_setpoint[1] = 20.0;
	right_setpoint[1] = 20.0;
	
	Quintic left_trajectory(left_values, left_setpoint, 0.0, 3.0);
	Quintic right_trajectory(right_values, right_setpoint , 0.0, 3.0);
	
	// Run through the control loop
	double t0, t;
	t0 = yarp::os::Time::now();
	do
	{
		t = yarp::os::Time::now() - t0;					// Elapsed time
		
		left_trajectory.getState(left_pos, left_vel, t);			// Get current desired state
		right_trajectory.getState(right_pos, right_vel,t);
		
		left_enc.getEncoders(left_values.data());				// Get current encoder values
		right_enc.getEncoders(right_values.data());
		
		for(int i = 0; i < n; i++)
		{
			left_ref[i]  = left_vel[i]  + 5.0*(left_pos[i]  - left_values[i]);
			left_ctrl.velocityMove(i, left_ref[i]);
			
			right_ref[i] = right_vel[i] + 5.0*(right_pos[i] - right_values[i]);
			right_ctrl.velocityMove(i, right_ref[i]);
		}
		
		yarp::os::Time::delay(0.01);
	}
	while(t < 3.0);
	
	// Ensure final values are zero
	for(int i = 0; i < n; i++)
	{
		left_ctrl.velocityMove(i, 0);
		right_ctrl.velocityMove(i,0);
	}	
}
