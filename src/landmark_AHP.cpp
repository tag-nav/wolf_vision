#include "landmark_AHP.h"

#include "state_homogeneous_3D.h"

namespace wolf {

/* Landmark - Anchored Homogeneous Point*/
LandmarkAHP::LandmarkAHP(Eigen::Vector4s _position, FrameBase* _frame, cv::Mat _2D_descriptor) :
    LandmarkBase(LANDMARK_CORNER, new StateHomogeneous3D(_position)), //TODO: Change "LANDMARK_CORNER"
    descriptor_(_2D_descriptor), anchor_frame_(_frame)
{
    std::cout << "LandmarkAHP p_ptr" << p_ptr_->getVector().transpose() << std::endl;
    setType("Image");
}

LandmarkAHP::~LandmarkAHP()
{
    //
}

} // namespace wolf
