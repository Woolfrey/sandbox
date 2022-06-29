    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                                 Code for testing math functions                                //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>                                                                                 // std::cout
#include <Math.h>

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
	
	if(get_qr_decomp(A, Q, R))
	{
		std::cout << "*************************************" << std::endl;
		std::cout << "* DEMONSTRATION OF QR DECOMPOSITION *" << std::endl;
		std::cout << "*************************************" << std::endl;
		
		std::cout << "\nWe decompose a matrix A = QR, where Q is orthogonal and R is upper-triangular." << std::endl;
		
		std::cout << "\nHere is the A matrix:" << std::endl;
		std::cout << A << std::endl;
		
		std::cout << "\nHere is the Q matrix:" << std::endl;
		std::cout << Q << std::endl;
		
		std::cout << "\nHere is Q*Q':" << std::endl;
		std::cout << Q*Q.transpose() << std::endl;
		
		std::cout << "\nThe determinant of Q*Q' is " << (Q*Q.transpose()).determinant() << "." << std::endl;
		
		std::cout << "\nHere is the R matrix:" << std::endl;
		std::cout << R << std::endl;
		
		// Set up a linear regression problem
		std::cout << "\n*********************" << std::endl;
		std::cout <<   "* LINEAR REGRESSION *" << std::endl;
		std::cout <<   "*********************" << std::endl;
		
		std::cout << "\nWe find a value for x that minimizes ||y - Ax||." << std::endl;
		std::cout << "This relies on the QR decomposition demonstrated above." << std::endl;
		
		A.col(0) = Eigen::VectorXd::Ones(A.rows());                                         // First coefficient is a constant
		Eigen::VectorXd x    = 100*Eigen::VectorXd::Random(n);                              // True coefficients
		Eigen::VectorXd xMax = 1e100*Eigen::VectorXd::Ones(n);                              // Upper bound for solution
		Eigen::VectorXd xMin = -xMax;                                                       // Lower bound for solution
		Eigen::VectorXd x0   = Eigen::VectorXd::Zero(n);                                    // Default value
		Eigen::VectorXd y    = A*x;                                                         // True values
			                        
		std::cout << "\nHere are the true values for x:" << std::endl;
		std::cout << x << std::endl;
		
		Eigen::VectorXd xHat = solve_qp(y,A,xMin,xMax,x0);
		std::cout << "\nHere are the optimised values for x from the QP solver:" << std::endl;
		std::cout << xHat << std::endl;
		
		// Test with noise / error
		   y = y + (y.norm()*0.05)*Eigen::VectorXd::Random(m);
		xHat = solve_qp(y,A,xMin,xMax,x0);
		
		std::cout << "\nNow we add 5"<<"%"<<" noise to the observations y and solve for x:" << std::endl;
		std::cout << xHat << std::endl;
		
		std::cout << "\nThe error ||y-A*xhat|| / ||y|| is " << (y - A*xHat).norm()/y.norm()*100 << "%." << std::endl;
		
		std::cout << "\nNow lets limit the largest value of x to 90%." << std::endl;
		
		double max = 0;
		for(int i = 0; i < n; i++)
		{
			if(abs(xHat(i)) > max) max = 0.90*abs(xHat(i));
		}
		
		xMax = max*Eigen::VectorXd::Ones(n);
		xMin = -xMax;
		xHat = solve_qp(y,A,xMin,xMax,x0);
			
		std::cout << "\nHere is the solution for x with limitations:" << std::endl;
		std::cout << xHat << std::endl;
		
		std::cout << "\nThe error ||y-A*xhat|| / ||y|| is " << (y - A*xHat).norm()/y.norm()*100 << "%."  <<std::endl;
	}	
	
	return 0;                                                                                   // No problems with main
}
