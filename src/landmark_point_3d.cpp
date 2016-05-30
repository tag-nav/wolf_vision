#include "landmark_point_3d.h"

namespace wolf {

LandmarkPoint3D::LandmarkPoint3D(StateBlock* _p_ptr, StateBlock* _o_ptr) :
    LandmarkBase(LANDMARK_CORNER, _p_ptr, _o_ptr)
{
    setType("Image");
}

LandmarkPoint3D::~LandmarkPoint3D()
{
    //
}

}
