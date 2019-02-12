
#include "base/landmark/landmark_corner_2D.h"

namespace wolf {

LandmarkCorner2D::LandmarkCorner2D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const Scalar& _aperture) :
	LandmarkBase("CORNER 2D", _p_ptr, _o_ptr)
{
  	setDescriptor(Eigen::VectorXs::Constant(1,_aperture));
}

LandmarkCorner2D::~LandmarkCorner2D()
{
    //
}

Scalar LandmarkCorner2D::getAperture() const
{
    return descriptor_(0);
}

} // namespace wolf
