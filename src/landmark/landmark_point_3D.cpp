#include "vision/landmark/landmark_point_3D.h"

namespace wolf {

LandmarkPoint3D::LandmarkPoint3D(Eigen::Vector3d _position, cv::Mat _2D_descriptor) :
    LandmarkBase("POINT 3D", std::make_shared<StateBlock>(_position, false)),
    descriptor_(_2D_descriptor)
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
