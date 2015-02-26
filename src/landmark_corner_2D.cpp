
#include "landmark_corner_2D.h"

LandmarkCorner2D::LandmarkCorner2D(const StateBaseShPtr & _p_ptr, const StateBaseShPtr & _o_ptr, const WolfScalar& _aperture) :
	LandmarkBase(LANDMARK_CORNER, _p_ptr, _o_ptr)
{
    if (_aperture!=0)
    	setDescriptor(Eigen::Vector1s(_aperture));
}

LandmarkCorner2D::~LandmarkCorner2D()
{
    //
}
