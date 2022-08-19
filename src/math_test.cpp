    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                                 Code for testing math functions                                //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>                                                                                 // std::cout
#include <Math.h>
#include <QPSolver.h>
#include <time.h>

clock_t timer;
double blah;

int m = 5;
int n = 5;

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                                MAIN                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
	// Set the size for the matrix
	if(argc == 2)                                                                               // Default for argc = 1
	{
		m = atoi(argv[1]);
		n = m;
	}
	else if(argc > 2)
	{
		m = atoi(argv[1]);
		n = atoi(argv[2]);
	}
	
	srand(time(NULL));                                                                          // Seed the random number generator
		
	Eigen::MatrixXd A = 100*Eigen::MatrixXd::Random(m,n);                                       // Generate random matrix within -100 to 100
	
	Eigen::MatrixXd Q, R;
	
	get_qr_decomposition(A,Q,R);
	
	return 0;                                                                                   // No problems with main
}
