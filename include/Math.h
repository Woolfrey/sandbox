#ifndef MATH_H_
#define MATH_H_

#include <Eigen/Dense>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Get the inverse of a matrix, avoid singularities as necessary               //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd get_inverse(const Eigen::MatrixXd &A)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> SVD(A, Eigen::ComputeFullU | Eigen::ComputeFullV);        // Get the SVD decomposition
	Eigen::MatrixXd V = SVD.matrixV();                                                          // V matrix
	Eigen::MatrixXd U = SVD.matrixU();                                                          // U matrix
	Eigen::VectorXd s = SVD.singularValues();                                                   // Get the singular values
	Eigen::MatrixXd invA(A.cols(), A.rows()); invA.setZero();                                   // Value we want to return
	
	for(int i = 0; i < A.cols(); i++)
	{
		for(int j = 0; j < s.size(); j++)
		{
			for(int k = 0; k < A.rows(); k++)
			{
				if(s(j) >= 1e-04)	invA(i,k) += (V(i,j)*U(k,j))/s(j);          // Fast inverse
//				else			invA(i,k) += 0;                             // Ignore singular directions
			}
		}
	}
	return invA;
}


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //           Decompose a matrix A in to orthogonal matrix Q and triangular matrix R              //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool get_qr_decomp(const Eigen::MatrixXd &A, Eigen::MatrixXd &Q, Eigen::MatrixXd &R)
{
	if(A.rows() < A.cols())
	{
		std::cerr << "[ERROR] get_qr_decomp() "
		          << "Number of rows in A matrix must be greater than or equal to the number of columns." << std::endl;
		
		return false;
	}
	else
	{
		Eigen::MatrixXd B = A;                                                              // Copy the input matrix as we need to modify it
		
		Q = Eigen::MatrixXd::Identity(A.rows(),A.rows());                                   // This will form the orthogonal matrix
		
		int m = A.rows();
		int steps = m-1; if(A.cols() < steps) steps = A.cols();
		
		Eigen::MatrixXd Qtemp(m,m);                                                         // Used to construct Q matrix
		Eigen::VectorXd b(B.rows());                                                        // For storing column vectors of B matrix
		Eigen::VectorXd v(B.rows());                                                        // Used for pivoting
		
		for(int i = 0; i < steps; i++)
		{
			Eigen::VectorXd e = Eigen::VectorXd::Zero(m-i); e(0) = 1;                   // Construct the pivot vector
			b = B.block(i,i,m-i,1);                                                     // Get the ith column of the B matrix
			v = (b - b.norm()*e).normalized();                                          // Compute the normalized vector along the pivot
			
			Qtemp.setIdentity();                                                        // Construct the ith orthogonal matrix
			Qtemp.block(i,i,m-i,m-i) = Eigen::MatrixXd::Identity(m-i,m-i) - 2*v*v.transpose();
			
			B = Qtemp*B;                                                                // Update the B matrix
			Q = Q*Qtemp.transpose();                                                    // Update the total orthogonal matrix
		}
		
		R = Q.transpose()*A;                                                                // Q'*A = Q'*Q*R = R
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Solve y = R*x, where R is an upper triangular matrix                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd solve_back_substitution(const Eigen::VectorXd &y,                                   // Vector of known values
                                        const Eigen::MatrixXd &R,                                   // Upper triangular matrix
                                        const Eigen::VectorXd &xMin,                                // Lower bound on solution
                                        const Eigen::VectorXd &xMax,                                // Upper bound on solution
                                        const Eigen::VectorXd &x0)                                  // Default solution if one cannot be found
{
	if(y.size() != R.rows() or R.rows() != R.cols())
	{
		std::cerr << "[ERROR] solve_back_substitution:"
		          << "Input vector of " << y.size() << " elements does not match triangular "
		          << "matrix with " << R.rows() << "x" << R.cols() << "elements." << std::endl;
		
		return x0;
	}
	else
	{
		Eigen::VectorXd x = Eigen::VectorXd::Zero(R.cols());                                // Value to be returned
		
		for(int i = R.cols()-1; i >= 0; i--)                                                // Start from the end and count back
		{

			// Sum up recursive values
			double sum = 0.0;
			for(int j = i; j < R.cols(); j++)
			{
				sum += R(i,j)*x(j);
			}
			
			if(R(i,i) == 0) x(i) = x0(i);                                               // Singular, so use default value
			else
			{
				x(i) = (y(i) - sum)/R(i,i);                                         // Solve for ith value                         
				
				     if(x(i) < xMin(i)) x(i) = xMin(i);                             // If below limit, override
				else if(x(i) > xMax(i)) x(i) = xMax(i);                             // If above limit, override
			}
		}
		
		return x;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Solve a system of equations y = Ax                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd solve_qp(const Eigen::VectorXd &y,                                                  // Vector of known values
                         const Eigen::MatrixXd &A,                                                  // Matrix that maps unknown to known values
                         const Eigen::MatrixXd &xMin,                                               // Lower bound on solution
                         const Eigen::MatrixXd &xMax,                                               // Upper bound on solution
                         const Eigen::MatrixXd &x0)                                                 // Default value if solution not found
{
	if(y.size() != A.rows())
	{
		std::cerr << "[ERROR] solve_qp(): "
		          << "y vector had " << y.size() << " elements but "
		          << "the A matrix had " << A.rows() << " rows." << std::endl;
		
		return x0;
	}
	else if(A.rows() < A.cols())
	{
		std::cerr << "[WARNING] solve_qp(): "
		          << "This system of equations is over-determined and cannot be solved." << std::endl;
		
		return x0;
	}
	else
	{
		// Solve the alternate system z = B*x
		
		Eigen::MatrixXd B = A.transpose()*A;
		Eigen::VectorXd z = A.transpose()*y;

		Eigen::MatrixXd Q, R;
		
		if(get_qr_decomp(B,Q,R)) return solve_back_substitution(Q.transpose()*z, R, xMin, xMax, x0);
		else
		{
			std::cerr << "[ERROR] solve_qp(): "
			          << "There was a problem with the QR decomposition!" << std::endl;
			
			return x0;
		}
	}
}

#endif

