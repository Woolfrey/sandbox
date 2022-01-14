#ifndef CUBICROT_H_
#define CUBICROT_H_

#include <Cubic.h>
#include <iDynTree/Core/Rotation.h>								// iDynTree::Rotation

class CubicRot : public Cubic
{
	public:
		CubicRot() : Cubic() {}								// Empty constructor
		
		CubicRot(const std::vector<iDynTree::Rotation> &points,				// Proper constructor
			const std::vector<double> &times);
			
		bool get_state(iDynTree::Rotation &rot,
				iDynTree::GeomVector3 &vel,
				iDynTree::GeomVector3 &acc,
				const double &time);
	
	private:
		std::vector<iDynTree::Rotation> R;						// Vector of waypoints

};												// Semicolon needed after a class declaration

CubicRot::CubicRot(const std::vector<iDynTree::Rotation> &points, const std::vector<double> &times)
		: Cubic()
		, R(points)
{
	// 1. Solve all differences in rotations: dR(i) = R(i)'*R(i+1)
	// 2. Convert all dR -> angle*axis
	// 3. Solve A*sdd = B*s
	// 4. Solve coefficients
}

bool CubicRot::get_state(iDynTree::Rotation &rot,
			iDynTree::GeomVector3 &vel,
			iDynTree::GeomVector3 &acc,
			const double &time)
{
	// 1. Figure out where we are on the trajectory
	// 2. Do the interpolation
	// 3. R(t) = R(0)*dR(t)
	
	return true;
}

#endif
