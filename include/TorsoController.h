#include <iCub/iKin/iKinFwd.h>							// iCub::iKin::iCubTorso object

class TorsoController
{
	public:
		TorsoController();							// Constructor
	
		void configure();							// Configure the device drivers
		
	private:
		int n = 3;
		
		yarp::sig::Vector q, qdot;						// Joint positions & velocities
		
		iCub::iKin::iCubTorso torso;
		
		// These interface with the hardware
    		yarp::dev::IControlLimits* limits;					// Joint limits?
		yarp::dev::IControlMode* mode;					// Sets the control mode of the motor
		yarp::dev::IEncoders* encoder;					// Joint position values (in degrees)
		yarp::dev::IVelocityControl* controller;				// Motor-level velocity controller
		yarp::dev::PolyDriver driver;						// Device driver
		
};											// Semicolon needed after class declaration

/******************** Constructor ********************/
TorsoController::TorsoController()
{
	// Worker bees can leave.
	// Even drones can fly away.
	// The Queen is their slave.
}

/******************** Configure the device drivers ********************/
void TorsoController::configure()
{
	this->q.resize(3);
	this->qdot.resize(3);
	
	// Configure the device driver
	yarp::os::Property options;							// Object to temporarily store options
	options.put("device", "remote_controlboard");				
	options.put("local", "/local/torso");						// Local port names
	options.put("remote", "/icubSim/torso");					// Where we want to connect to

	this->driver.open(options);							// Assign the options to the driver
	if(!this->driver.isValid())
	{
		yError("Unable to configure device. Here are the known devices:");
		yInfo() <<  yarp::dev::Drivers::factory().toString().c_str();	
	}		
	else				yInfo() << "Successfully configured device driver for the torso.";
	
	// Then configure the joint controllers
	if(!this->driver.view(this->controller))
	{
		yError() << "Unable to configure the controller for the torso.";
		this->driver.close();
	}
	else if(!this->driver.view(this->mode)) yError() << "Unable to configure the control mode for the torso.";
	else
	{		
		for(int i = 0; i < this->n; i++)
		{
			this->mode->setControlMode(i, VOCAB_CM_VELOCITY);				// Set the motor in velocity mode
			this->controller->setRefAcceleration(i,std::numeric_limits<double>::max());	// CHANGE THIS?
			this->controller->velocityMove(i,0.0);					// Ensure initial velocity is zero
		}
		yInfo() << "Successfully configured velocity control of the torso.";
	}
	
	// Try and obtain the joint limits
	if(!this->driver.view(this->limits)) 	yError() << "Unable to obtain the joint limits for the torso.";
	else					yInfo() << "Successfully obtained limits for the torso.";

	// Finally, configure the encoders
	if(this->driver.view(this->encoder))
	{		
		for(int i = 0; i < 5; i++)
		{
			if(this->encoder->getEncoders(this->q.data()))		// Get initial encoder values
			{
				yInfo() << "Successfully configured the encoder for the torso.";
				break;							// This should exit the for-loop
			}
			if(i == 5) yError() << "Could not obtain encoder values for the torso sin 5 attempts.";		
			yarp::os::Time::delay(0.02);					// Wait a little bit and try again
		}
	}
}
