

/*************************************** Arm Controller ***********************************************/
class ArmController
{
	public:
		ArmController();						// Constructor
		
		home();							// Return to a default position
		
		moveToConfiguration(const yarp::sig::Vector &target);	// Move the joints to a given configuration
	
	private:

		// Properties
		yarp::dev::PolyDriver driver;					// Device driver
		yarp::dev::IEncoders* encoder;				// Reads encoder values
		yarp::dev::IControlMode* mode;				// Sets the control mode of the motor
		yarp::dev::IVelocityControl* controller;			// Motor-level velocity controller
		
		// Functions
		bool configureControl();
		bool configureDriver();
		bool configureEncoder();
};										// Need this after a class declaration

