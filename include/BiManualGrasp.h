    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                                A class for bimanual grasping                                   //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef BIMANUALGRASP_H_
#define BIMANUALGRASP_H_

#include <Humanoid.h>
#include <Payload.h>

class BiManualGrasp : public Humanoid
{
	public:
		BiManualGrasp(const std::string &fileName);
		
		bool grasp_object(const iDynTree::Transform &left,
		                  const iDynTree::Transform &right,
		                  const iDynTree::Transform &object);
		                  
		bool release_object();
		
	private:
		bool payloadExists;
		
		CartesianTrajectory payloadTrajectory;                                              // As it says on the label
	
		Eigen::Matrix<double,6,12> G;                                                       // Grasp matrix
		Eigen::Matrix<double,12,6> C;                                                       // Constraint matrix
		
		Payload payload;                                                                    // Object payload
		
		// Functions related to PeriodicThread class
		bool threadInit() {return true;}
		void run() { std::cout << "It's doing things here!" << std::endl;}
		void threadRelease() {}
		
		
};                                                                                                  // Semicolon needed after a class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
BiManualGrasp::BiManualGrasp(const std::string &fileName) :
                             Humanoid(fileName)
{

}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Grasp an object                                             //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool BiManualGrasp::grasp_object(const iDynTree::Transform &left,
                                 const iDynTree::Transform &right,
                                 const iDynTree::Transform &object)
{
	// 1. Move hands to grasp location with Humanoid::move_to_pose(left, right);
	
	// 2. Set up grasp and constraint matrices G, C
	
	// 3. Run force control
	
	std::cout << "Success!" << std::endl;
	
	return true;
}

#endif
