#ifndef MATH_H_
#define MATH_H_

#include <iostream>
#include <Eigen/Dense>

bool get_lu_decomposition(const Eigen::MatrixXd &A,                                                 // Split a matrix A in to L and U
                                Eigen::MatrixXd &L,
                                Eigen::MatrixXd &U);
                                
bool get_qr_decomposition(const Eigen::MatrixXd &A,                                                 // Split a matrix in to Q and R
                                Eigen::MatrixXd &Q,
                                Eigen::MatrixXd &R);
                                                                                              
Eigen::MatrixXd get_cholesky_decomposition(const Eigen::MatrixXd &A);                               // A = L*L' for a positive-definite matrix

Eigen::MatrixXd get_cholesky_inverse(const Eigen::MatrixXd &A);                                     // Get the inverse of a positive-definite matrix

Eigen::MatrixXd get_inverse(const Eigen::MatrixXd &A);                                              // Get inverse via LU decomposition

Eigen::MatrixXd get_pseudoinverse(const Eigen::MatrixXd &A);

Eigen::MatrixXd get_pseudoinverse(const Eigen::MatrixXd &A,
                                  const Eigen::MatrixXd &W);

Eigen::MatrixXd get_triangular_inverse(const Eigen::MatrixXd &T);                                   // Get the inverse of a triangular matrix

Eigen::VectorXd backward_substitution(const Eigen::VectorXd &y,                                     // Solve a system for upper-triangular U
                                      const Eigen::MatrixXd &U,
                                      const Eigen::VectorXd &x0);
                                  
Eigen::VectorXd forward_substitution(const Eigen::VectorXd &y,                                      // Solve a sytem for lower-triangular L
                                     const Eigen::MatrixXd &L,
                                     const Eigen::VectorXd &x0);

Eigen::VectorXd solve_cholesky_system(const Eigen::VectorXd &y,                                     // Solve y = A*x where A is positive-definite
                                      const Eigen::MatrixXd &A);

Eigen::VectorXd solve_linear_system(const Eigen::VectorXd &y,                                       // Solve y = A*x for arbitrary A
                                    const Eigen::MatrixXd &A,
                                    const Eigen::VectorXd &x0);

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                               Decompose a matrix A = LU                                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool get_lu_decomposition(const Eigen::MatrixXd &A,
                                Eigen::MatrixXd &L,
                                Eigen::MatrixXd &U)
{
	int n = A.cols();
	
	if(A.rows() != A.cols())
	{
		std::cerr << "[ERROR] get_lu_decomposition(): "
                          << "Expected a square matrix for A, but "
                          << "it was " << A.rows() << "x" << A.cols() << "." << std::endl;
                
                return false;
        }
        else
        {
        	// Doolittle Algorithm - assume diagonal elements of L are all 1
        	L = Eigen::MatrixXd::Identity(n,n);
        	U = Eigen::MatrixXd::Zero(n,n);
        	
        	
		for(int i = 0; i < n; i++)
		{
			for(int j = 0; j < n; j++)
			{
				if(i == j and abs(A(i,j)) <= 1E-6)
				{
					std::cerr << "[ERROR] get_lu_decomposition(): "
					          << "A diagonal element of matrix A is close to zero! "
					          << "LU decomposition will fail (we need to program in pivoting)." << std::endl;
					
					return false;
				}
				float sum = 0.0;			
				for(int k = 0; k < std::min(i,j); k++) sum += L(i,k)*U(k,j);
				
				if(i > j)
				{
					if(abs(U(j,j)) < 1E-6) L(i,j) = 0.0;
					else                   L(i,j) = (A(i,j) - sum)/U(j,j);
				}
				else U(i,j) = (A(i,j) - sum)/L(i,i);
			}
        	}
        	
        	return true;
        }
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //           Decompose a matrix A in to orthogonal matrix Q and triangular matrix R              //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool get_qr_decomposition(const Eigen::MatrixXd &A,
                                Eigen::MatrixXd &Q,
                                Eigen::MatrixXd &R)
{
	int m = A.rows();
	int n = A.cols();
	
	if(m < n)
	{
		std::cerr << "[ERROR] get_qr_decomp() "
		          << "Matrix A has " << A.rows() << " rows which is less than its "
		          << A.cols() << " columns. Cannot solve the QR decomposition." << std::endl;
		
		return false;
	}
	else
	{
		// Schwarz-Rutishauser Algorithm.
		// A full decomposition of an mxn matrix A (m > n) is:
		//    [ Qr Qn ][ R ]  = A
		//             [ 0 ]
		//
		// where: - Qr is mxn,
		//        - Qn is mx(m-n)
		//        - R  is nxn
		//
		// The null space of A is obtained with N = Qn*Qn'.
		// This algorithm returns only Qr, and R for efficiency.
		
		Q = A;
		R = Eigen::MatrixXd::Zero(n,n);
		
		for(int j = 0; j < n; j++)
		{
			for(int i = 0; i < j; i++)
			{
				R(i,j)   = Q.col(i).dot(Q.col(j));                                  // Project the columns
				Q.col(j) = Q.col(j) - R(i,j)*Q.col(i);
			}
			
			R(j,j) = Q.col(j).norm();
			
			if(abs(R(j,j)) > 1E-6) Q.col(j) /= R(j,j);
			else                   Q.col(j).setZero();
		}
		
		return true;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Decompose a positive-definite matrix A in to L*L'                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd get_cholesky_decomposition(const Eigen::MatrixXd &A)
{
	int n = A.rows();
	Eigen::MatrixXd L = Eigen::MatrixXd::Zero(n,n);
	if(A.rows() != A.cols())
	{
		std::cerr << "[ERROR] get_cholesky_decomp(): "
		          << "Expected a square matrix, but "
		          << "the input was " << A.rows() << "x" << A.cols() << "." << std::endl;
	}
	else
	{
                // Cholesky–Banachiewicz algorithm (row-wise)
		for(int i = 0; i < n; i++)
		{
			for(int j = 0; j <= i; j++)
			{				
				float sum = 0.0;
				for(int k = 0; k <= j; k++) sum += L(i,k) * L(j,k);
				
				if(i == j) 
				{
					float temp = A(i,j) - sum;
					if(temp < 0)
					{
						std::cerr << "\n[ERROR] get_cholesky_decomp(): "
						          << "The input matrix is singular and can't be decomposed.\n" << std::endl;
						
						return L;
					}
					else L(i,j) = sqrt(A(i,j) - sum);
				}
				else         L(i,j) = (A(i,j) - sum)/L(j,j);
			}
		}
	}
	
	return L;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                          Get the inverse of a positive-definite matrix                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd get_cholesky_inverse(const Eigen::MatrixXd &A)
{
	int n = A.rows();
	
	if(A.cols() != n)
	{
		std::cerr << "[ERROR] get_cholesky_inverse(): "
		          << "Expected a square matrix but "
		          << "the input was " << A.rows() << "x" << A.cols() << "." << std::endl;
		
		return Eigen::MatrixXd::Zero(n,n);
	}
	else
	{
		Eigen::MatrixXd L = get_cholesky_decomposition(A);                                  // As it says on the label
		Eigen::MatrixXd I = get_triangular_inverse(L);                                      // Fast inverse of a triangular matrix
		
		return I.transpose()*I;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Get the inverse of a matrix with LU decomposition                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd get_inverse(const Eigen::MatrixXd &A)
{
	std::cerr << "[ERROR] get_inverse(): This function needs reprogramming." << std::endl;
	return Eigen::MatrixXd::Zero(A.cols(),A.rows());
/*
	int n = A.rows();
	if(A.cols() != n)
	{
		std::cerr << "[ERROR] get_inverse(): "
		          << "Expected a square matrix, but it was "
		          << A.rows() << "x" << A.cols() << ". "
		          << "Did you mean to call get_pseudoinverse()?" << std::endl;
		          
		return Eigen::MatrixXd::Zero(n,n);
	}
	else
	{
		Eigen::MatrixXd L, U;
		if(get_lu_decomposition(A,L,U))
		{
			return get_triangular_inverse(U)*get_triangular_inverse(L);
		}
		else    return Eigen::MatrixXd::Zero(n,n);
	}
*/
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                         Get the pseudoinverse of a rectangular matrix                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd get_pseudoinverse(const Eigen::MatrixXd &A)
{
	int m = A.rows();
	int n = A.cols();
	
	if(m > n)      return (A.transpose()*A).inverse()*A.transpose();                            // (A'*A)^-1*A'
	else if(m < n) return A.transpose()*(A*A.transpose()).inverse();                            // A'*(A*A')^-1
	else           return A.inverse();                                                          // A^-1
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Get the weighted pseudoinverse of a rectangular matrix                       //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd get_pseudoinverse(const Eigen::MatrixXd &A, const Eigen::MatrixXd &W)
{
	// Variables in this scope
	int m = A.rows();
	int n = A.cols();
	
	if(m > n)
	{
		if(W.rows() != m or W.cols() != n)
		{
			std::cerr << "[ERROR] get_pseudoinverse(): "
			          << "Expected a " << m << "x" << m << "weighting matrix, "
			          << "but it was " << W.rows() << "x" << W.cols() << "." << std::endl;
			
			return Eigen::MatrixXd::Zero(n,m);
		}
		else
		{
			Eigen::MatrixXd invW = W.inverse();
			return (A.transpose()*invW*A).inverse()*A.transpose()*invW;
		}
	}
	else if(m < n)
	{
		if(W.rows() != n or W.cols() != n)
		{
			std::cerr << "[ERROR] get_pseudoinverse(): "
			          << "Expected a " << n << "x" << n << "weighting matrix, "
			          << "but it was " << W.rows() << "x" << W.cols() << "." << std::endl;
			
			return Eigen::MatrixXd::Zero(n,m);
		}
		else
		{
			Eigen::MatrixXd invW = W.inverse();
			return invW*A.transpose()*(A*A.transpose()).inverse();
		}
	}
	else return A.inverse();
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                            Get the inverse of a triangular matrix                             //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd get_triangular_inverse(const Eigen::MatrixXd &T)
{
	int n = T.rows();
	if(T.cols() != n)
	{
		std::cerr << "[ERROR] get_triangular_inverse(): "
		          << "Expected a square matrix for the input but it was "
		          << T.rows() << "x" << T.cols() << "." << std::endl;
		
		return Eigen::MatrixXd::Zero(n,n);
	}
	else
	{
		Eigen::MatrixXd I = Eigen::MatrixXd::Zero(n,n);                                     // Value to be returned
		
		// Lower triangular
		if(T(0,n-1) == 0)
		{
			for(int i = 0; i < n; i++)                                                  // Start from first row
			{
				for(int j = 0; j <= i; j++)                                         // Move forward through columns
				{
						if(i == j) I(i,i) = 1/T(i,i);
						else
						{
							float sum = 0.0;
							for(int k = j; k < i; k++) sum += T(i,k)*I(k,j);
							
							I(i,j) = -sum/T(i,i);
						}
				}
			}
		}
		// Upper triangular
		else if(T(n-1,0) == 0)                                                              // Start from last row
		{
			for(int i = n-1; i >= 0; i--)
			{
				for(int j = n-1; j >= i; j--)                                       // Move backward through columns
				{
					if(i == j)
					{
						if(abs(T(i,i)) < 1E-5) I(i,i) = 0.0;
						else                   I(i,i) = 1/T(i,i);
					}
					else
					{
						float sum = 0.0;
						for(int k = i+1; k <= j; k++) sum += T(i,k)*I(k,j);
						
						if(abs(T(i,i)) < 1E-5) I(i,j) = 0.0;
						else                   I(i,j) = -sum/T(i,i);
					}
				}
			}
		}
		// Not triangular
		else
		{
			std::cerr << "[ERROR] triangular_inverse(): "
			          << "The input matrix does not appear to be triangular "
			          << "as neither of the corner elements are zero!" << std::endl; 
		}

		return I;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Solve y = U*x, where U is an upper-triangular matrix                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd backward_substitution(const Eigen::VectorXd &y,
                                      const Eigen::MatrixXd &U,
                                      const Eigen::VectorXd &x0)
{
	int n = U.cols();
	if(y.size() != n or U.rows() != n or x0.size() != n)
	{
		std::cerr << "[ERROR] back_substitution(): "
		          << "Dimensions of inputs do not match. "
		          << "The y vector had " << y.size() << " elements, "
		          << "the U matrix had " << U.rows() << "x" << U.cols() << " elements, and "
		          << "the x0 vector had " << x0.size() << " elements." << std::endl;
		          
		return x0;
	}
	else
	{
		Eigen::VectorXd x(n);
		
		for(int i = n-1; i >=0; i--)                                                        // Start from the end and count backwards
		{
			float sum = 0.0;
			for(int j = i+1; j < n; j++) sum += U(i,j)*x(j);
			
			if(abs(U(i,i)) < 1E-6) x(i) = x0(i);
			else                   x(i) = (y(i)-sum)/U(i,i);
		}
		return x;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Solve y = L*x, where L is an lower-triangular matrix                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd forward_substitution(const Eigen::VectorXd &y,
                                     const Eigen::MatrixXd &L,
                                     const Eigen::VectorXd &x0)
{
	int n = L.cols();
	
	if(y.size() != n or L.rows() != n or x0.size() != n)
	{
		std::cerr << "[ERROR] forward_substitution(): "
		          << "Dimensions of inputs do not match. "
		          << "The y vector had " << y.size() << " elements, "
		          << "the L matrix had " << L.rows() << "x" << L.cols() << " elements, and "
		          << "the x0 vector had " << x0.size() << " elements." << std::endl;
		
		return x0;
	}
	else
	{
		Eigen::VectorXd x(n);
		
		for(int i = 0; i < n; i++)
		{
			float sum = 0.0;
			for(int j = 0; j < i; j++) sum += L(i,j)*x(j);
			
			if(L(i,i) < 1E-6) x(i) = x0(i);
			else              x(i) = (y(i) - sum)/L(i,i);
		}
		
		return x;
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //        Solve a linear system of equations given by y = Ax where A is positive-definite        //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd solve_cholesky_system(const Eigen::VectorXd &y,
                                      const Eigen::MatrixXd &A)
{
	// NOTE: This function does not need a default value x0
	// since by definition A cannot be singular and a solution *must* exist.
	
	int n = A.rows();
	if(A.cols() != n)
	{
		std::cerr << "[ERROR] solve_cholesky_system(): "
		          << "Expected a square A matrix but it was "
		          << A.rows() << "x" << A.cols() << "." << std::endl;
		          
		return Eigen::VectorXd::Zero(n);
	}
	else if(y.size() != n)
	{
		std::cerr << "[ERROR] solve_cholesky_system(): "
		          << "Expected a " << n << "x1 vector for y, but "
		          << "it was " << y.size() << "x1." << std::endl;
		
		return Eigen::VectorXd::Zero(n);
	}
	else
	{
		Eigen::MatrixXd    L = get_cholesky_decomposition(A);
		Eigen::VectorXd zero = Eigen::VectorXd::Zero(n);
		return backward_substitution(forward_substitution(y,L,zero), L.transpose(),zero); // lol too easy
	}
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Solve a linear system of equations given by y = Ax                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd solve_linear_system(const Eigen::VectorXd &y,
                                    const Eigen::MatrixXd &A,
                                    const Eigen::VectorXd &x0)
{
	int m = A.rows();
	int n = A.cols();
	
	// Check that the inputs are sound
	if(y.size() != m or x0.size() != n)
	{
		std::cerr << "[ERROR] solve_linear_system(): "
		          << "Dimensions of inputs do not match! "
		          << "The y vector was " << y.size() << "x1 ,"
		          << "the A matrix was " << m << "x" << n << ", and "
		          << "the x0 vector was " << x0.size() << "x1." << std::endl;
		
		return x0;
	}
	else
	{
		if(m == n)
		{
			Eigen::MatrixXd L, U;
			get_lu_decomposition(A,L,U);
			return backward_substitution(forward_substitution(y,L,Eigen::VectorXd::Zero(n)),U,x0);
		}
		else if(m > n)
		{
			Eigen::MatrixXd At = A.transpose();
			Eigen::MatrixXd L, U;
			get_lu_decomposition(At*A,L,U);
			return backward_substitution(forward_substitution(At*y,L,Eigen::VectorXd::Zero(n)),U,x0);
		}
		else
		{
			// M = [ 0  A ]
			//     [ A' W ]
			Eigen::MatrixXd At = A.transpose();
			Eigen::MatrixXd M  = Eigen::MatrixXd::Zero(m+n,m+n);
//			M.block(0,0,m,m)   = Eigen::MatrixXd::Zero(m,m);
			M.block(0,m,m,n)   = A;
			M.block(m,0,n,m)   = A.transpose();
			M.block(m,m,n,n)   = Eigen::MatrixXd::Identity(n,n);
			
			Eigen::MatrixXd Q, R;
			get_qr_decomposition(M,Q,R);
			
			Eigen::MatrixXd R22 = R.block(m,m,n,n);
			Eigen::MatrixXd Q12 = Q.block(0,m,m,n);
			return backward_substitution(Q12.transpose()*y,R22,x0);
			
		}
	}
}
#endif                                    
