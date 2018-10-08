#include "landmark_point_3D.h"

namespace wolf {

LandmarkPoint3D::LandmarkPoint3D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, Eigen::Vector3s _position, cv::Mat _2D_descriptor) :
    LandmarkBase("POINT 3D", _p_ptr, _o_ptr),
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
