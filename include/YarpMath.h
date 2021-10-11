#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

/******************** Return the norm of a vector ********************/
float norm(const yarp::sig::Vector &vector)
{
	float norm = 0.0;
	
	for(int i = 0; i < vector.size(); i++) norm += vector[i]*vector[i];
	
	return sqrt(norm);
}

/******************** Normalise a vector ********************/
yarp::sig::Vector normalised(yarp::sig::Vector vector)
{
	float denom = norm(vector);
	
	for(int i = 0; i < vector.size(); i++) vector[i] /= denom;		// Divide by norm

	return vector;
}

/******************** Normalise a vector ********************/
float dot_product(const yarp::sig::Vector &v1, const yarp::sig::Vector &v2)
{
	if(v1.size() != v2.size())
	{
		// ERROR: Vectors must be of the same length
		return 0.0;
	}
	else
	{
		float dot = 0.0;
		for(int i = 0; i < v1.size(); i++) dot += v1[i]*v2[i];
		return dot;	
	}
}

/********************* Invert a matrix, automatically adds damping *********************/
yarp::sig::Matrix inverse(const yarp::sig::Matrix &A)
{
	int m = (int)A.rows();
	int n = (int)A.cols();
	
	yarp::sig::Matrix U(m,m);							// Left orthogonal matrix
	yarp::sig::Vector s(m);							// Vector of singular values
	yarp::sig::Matrix V(n,n);							// Right orthogonal matrix
	yarp::sig::Matrix invA(n,m);							// Inverse to be returned
	
	yarp::math::SVD(A,U,s,V);							// Get the singular value decomposition
	
	for(int i = 0; i < n; i++)
	{
		for(int j = 0; j < m; j++)
		{
			if(s(j) > 1.0e-4)	invA(i,j) = V(i,j)/s(j);		// This circumvents the off-diagonal elements for V*S^-1
			else			invA(i,j) = V(i,j)*1.0e7;		// Singular, so saturate the value
		}
	}
	
	return invA *= U.transposed();						// Now return (V*S^-1)*U^T
}

/********************* Invert a diagonal matrix - avoids SVD  *********************/
yarp::sig::Matrix diagonal_inverse(const yarp::sig::Matrix &A)
{
	if((int)A.rows() != (int)A.cols())
	{
		// ERROR: This matrix isn't square!
	}

	int m = (int)A.rows();								// Number of rows & columns
	yarp::sig::Matrix invA(m,m);							// Matrix to be returned
	
	for(int i = 0; i < m; i++)
	{
		for(int j = 0; j < m; j++)
		{
			if(A(i,j) > 1.0e-4)	invA(i,j) = 1.0/A(i,j);		// Standard inversion
			else			invA(i,j) = A(i,j)*1.0e7;		// Singular, so saturate the value
		}
	}
	
	return invA;
}

/********************* Get the Axis-Angle from a 3x3 matrix  *********************/
yarp::sig::Vector get_axisAngle(const yarp::sig::Matrix &R)
{

}
