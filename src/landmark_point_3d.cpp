#include "landmark_point_3d.h"

namespace wolf {

LandmarkPoint3D::LandmarkPoint3D(StateBlock* _p_ptr, StateBlock* _o_ptr, Eigen::Vector3s _position, cv::Mat _2D_descriptor) :
    LandmarkBase(LANDMARK_CORNER, "POINT 3D", _p_ptr, _o_ptr),
    descriptor_(_2D_descriptor),
    position_(_position)
{
    //LandmarkPoint3D* landmark_ptr = (LandmarkPoint3D*)_p_ptr;
//    position_ =
//    descriptor_ = _2D_descriptor;
}

LandmarkPoint3D::~LandmarkPoint3D()
{
    //
}

}
