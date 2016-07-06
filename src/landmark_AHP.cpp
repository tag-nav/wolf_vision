#include "landmark_AHP.h"

#include "state_homogeneous_3D.h"

namespace wolf {

/* Landmark - Anchored Homogeneous Point*/
LandmarkAHP::LandmarkAHP(StateBlock* _p_ptr, StateBlock* _o_ptr, cv::Mat _2D_descriptor,
                         Eigen::Vector4s _position, FrameBase* _frame) :
    LandmarkBase(LANDMARK_CORNER, _p_ptr, new StateBlock(3)), //TODO: Change "LANDMARK_CORNER"
    descriptor_(_2D_descriptor)
{
    setType("Image"); //TODO: ¿?
    position = new StateHomogeneous3D(_position);
    anchorRobot_ = _frame->getPPtr();
}

LandmarkAHP::~LandmarkAHP()
{
    //
}

} // namespace wolf
