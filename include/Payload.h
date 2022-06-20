    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                    A class for describing an object carried by a robot                         //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef PAYLOAD__H_
#define PAYLOAD_H_

#include<iDynTree/Core/Transform.h>

class Payload
{
	public:
		Payload();
		
	private:
	
		iDynTree::SpatialInertia inertia;
		iDynTree::Transform      pose;
		iDynTree::Twist          velocity;
};                                                                                                  // Semicolon needed after a class declaration

#endif
