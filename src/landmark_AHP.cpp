#include "landmark_AHP.h"

#include "state_homogeneous_3D.h"
#include "factory.h"

namespace wolf {

/* Landmark - Anchored Homogeneous Point*/
LandmarkAHP::LandmarkAHP(Eigen::Vector4s _position, FrameBase* _anchor_frame, SensorBase* _anchor_sensor, cv::Mat _2D_descriptor) :
    LandmarkBase(LANDMARK_AHP, new StateHomogeneous3D(_position)),
    descriptor_(_2D_descriptor), anchor_frame_(_anchor_frame), anchor_sensor_(_anchor_sensor)
{
//    std::cout << "LandmarkAHP p_ptr" << p_ptr_->getVector().transpose() << std::endl;
    setType("AHP");
}

LandmarkAHP::~LandmarkAHP()
{
    //
}

} // namespace wolf
