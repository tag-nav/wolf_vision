#include "vision/landmark/landmark_point_3d.h"

namespace wolf {

LandmarkPoint3d::LandmarkPoint3d(Eigen::Vector3d _position, cv::Mat _2d_descriptor) :
    LandmarkBase("LandmarkPoint3d", std::make_shared<StateBlock>(_position, false)),
    descriptor_(_2d_descriptor)
{
    //LandmarkPoint3d* landmark_ptr = (LandmarkPoint3d*)_p_ptr;
//    position_ =
//    descriptor_ = _2d_descriptor;
}

LandmarkPoint3d::~LandmarkPoint3d()
{
    //
}

}
