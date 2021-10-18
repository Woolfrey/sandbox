#include <Haiku.h>
#include <yarp/os/Network.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

class Threader : public yarp::os::PeriodicThread					// Inherits the PeriodicThread class
{
	public:
		Threader(double period):yarp::os::PeriodicThread(period) {}

		bool threadInit()							// Executed before run()
		{
			return true;
		}
		
		void run()								// Executed every time period afer ControlThread.start() is called
		{
			haiku();
			std::cout << "." << std::endl;
			std::cout << "." << std::endl;
			std::cout << "." << std::endl;
		}
		
		void threadRelease()							// Things done when stop() is called
		{
		
		}
};											// Needed after a class declaration



int main(int argc, char *argv[])
{
	yarp::os::Network yarp;
	
	if(!yarp.checkNetwork())
	{
		yError("No yarp network was found. Have you started it yet?");
		return 1;
	}
	
	Threader stuff(3.0);								// Create threader with 3.0 second period
	stuff.start();
	
	double start_time = yarp::os::Time::now();
	do
	{
	}while(yarp::os::Time::now() - start_time < 9.0);
	
	stuff.stop();
	
	return 0;									// No problems with main
}



 /* THIS IS THE CODE DOWNLOADED FROM THE ICUB TUTORIALS
 // -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
  
 #include <yarp/os/Network.h>
 #include <yarp/os/PeriodicThread.h>
 #include <yarp/os/Time.h>
 #include <yarp/os/Property.h>
 #include <yarp/dev/ControlBoardInterfaces.h>
 #include <yarp/dev/PolyDriver.h>
 #include <yarp/sig/Vector.h>
  
 #include <string>
 #include <iostream>
  
 using namespace yarp::os;
 using namespace yarp::dev;
 using namespace yarp::sig;
  
 using namespace std;
  
 class ControlThread: public PeriodicThread
 {
     PolyDriver dd;
     IVelocityControl *ivel;
     IEncoders        *iencs;
     Vector encoders;
     Vector commands;
     int count;
 public:
     ControlThread(double period):PeriodicThread(period){}
  
     bool threadInit()
     {
         //initialize here variables
         printf("ControlThread:starting\n");
         
         Property options;
         options.put("device", "remote_controlboard");
         options.put("local", "/local/head");
         
         //substitute icubSim with icub for use with the real robot
         options.put("remote", "/icubSim/head");
         
         dd.open(options);
  
         dd.view(iencs);
         dd.view(ivel);
  
         if ( (!iencs) || (!ivel) )
             return false;
         
         int joints;
    
         iencs->getAxes(&joints);
     
         encoders.resize(joints);
         commands.resize(joints);
  
         commands=10000;
         ivel->setRefAccelerations(commands.data());
  
         count=0;
         return true;
     }
  
     void threadRelease()
     {
         printf("ControlThread:stopping the robot\n");
         
         ivel->stop();
  
         dd.close();
  
         printf("Done, goodbye from // -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
  
 #include <yarp/os/Network.h>
 #include <yarp/os/PeriodicThread.h>
 #include <yarp/os/Time.h>
 #include <yarp/os/Property.h>
 #include <yarp/dev/ControlBoardInterfaces.h>
 #include <yarp/dev/PolyDriver.h>
 #include <yarp/sig/Vector.h>
  
 #include <string>
 #include <iostream>
  
 using namespace yarp::os;
 using namespace yarp::dev;
 using namespace yarp::sig;
  
 using namespace std;
  
 class ControlThread: public PeriodicThread
 {
     PolyDriver dd;
     IVelocityControl *ivel;
     IEncoders        *iencs;
     Vector encoders;
     Vector commands;
     int count;
 public:
     ControlThread(double period):PeriodicThread(period){}
  
     bool threadInit()
     {
         //initialize here variables
         printf("ControlThread:starting\n");
         
         Property options;
         options.put("device", "remote_controlboard");
         options.put("local", "/local/head");
         
         //substitute icubSim with icub for use with the real robot
         options.put("remote", "/icubSim/head");
         
         dd.open(options);
  
         dd.view(iencs);
         dd.view(ivel);
  
         if ( (!iencs) || (!ivel) )
             return false;
         
         int joints;
    
         iencs->getAxes(&joints);
     
         encoders.resize(joints);
         commands.resize(joints);
  
         commands=10000;
         ivel->setRefAccelerations(commands.data());
  
         count=0;
         return true;
     }
  
     void threadRelease()
     {
         printf("ControlThread:stopping the robot\n");
         
         ivel->stop();
  
         dd.close();
  
         printf("Done, goodbye from ControlThread\n");
     }
  
     void run()
     {
         //do the work
         iencs->getEncoders(encoders.data());
  
         count++;
  
         if (count%2)
             commands=5;
         else
             commands=-5;
     
         ivel->velocityMove(commands.data());
  
         printf(".");
     }
 };
  
 int main(int argc, char *argv[]) 
 {
     Network yarp;
  
     if (!yarp.checkNetwork())
     {
         printf("No yarp network, quitting\n");
         return 1;
     }
  
  
     ControlThread myThread(4.0); //period is 4s
  
     myThread.start();
  
     bool done=false;
     double startTime=Time::now();
     while(!done)
     {
         if ((Time::now()-startTime)>5)
             done=true;
     }
     
     myThread.stop();
  
     return 0;
 } ControlThread\n");
     }
  
     void run()
     {
         //do the work
         iencs->getEncoders(encoders.data());
  
         count++;
  
         if (count%2)
             commands=5;
         else
             commands=-5;
     
         ivel->velocityMove(commands.data());
  
         printf(".");
     }
 };
  
 int main(int argc, char *argv[]) 
 {
     Network yarp;
  
     if (!yarp.checkNetwork())
     {
         printf("No yarp network, quitting\n");
         return 1;
     }
  
  
     ControlThread myThread(4.0); //period is 4s
  
     myThread.start();
  
     bool done=false;
     double startTime=Time::now();
     while(!done)
     {
         if ((Time::now()-startTime)>5)
             done=true;
     }
     
     myThread.stop();
  
     return 0;
 }*/
