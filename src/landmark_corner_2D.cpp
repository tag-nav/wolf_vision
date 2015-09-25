
#include "landmark_corner_2D.h"

LandmarkCorner2D::LandmarkCorner2D(StateBase* _p_ptr, StateOrientation* _o_ptr, const WolfScalar& _aperture) :
	LandmarkBase(LANDMARK_CORNER, _p_ptr, _o_ptr)
{
  	setDescriptor(Eigen::VectorXs::Constant(1,_aperture));
}

LandmarkCorner2D::~LandmarkCorner2D()
{
    //
}


WolfScalar LandmarkCorner2D::getAperture() const
{
    return descriptor_(0);
}
