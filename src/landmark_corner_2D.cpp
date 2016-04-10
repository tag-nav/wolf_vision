
#include "landmark_corner_2D.h"

namespace wolf {

LandmarkCorner2D::LandmarkCorner2D(StateBlock* _p_ptr, StateBlock* _o_ptr, const WolfScalar& _aperture) :
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

} // namespace wolf
