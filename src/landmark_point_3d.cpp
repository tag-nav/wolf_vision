#include "landmark_point_3d.h"

namespace wolf {

LandmarkPoint3D::LandmarkPoint3D(StateBlock* _p_ptr, StateBlock* _o_ptr, Eigen::Vector3s _position, cv::Mat _2D_descriptor) :
    LandmarkBase(LANDMARK_CORNER, _p_ptr, _o_ptr),
    position_(_position),
    descriptor_(_2D_descriptor)
{
    setType("Image");
    //LandmarkPoint3D* landmark_ptr = (LandmarkPoint3D*)_p_ptr;
//    position_ =
//    descriptor_ = _2D_descriptor;
}

LandmarkPoint3D::~LandmarkPoint3D()
{
    //
}

}
