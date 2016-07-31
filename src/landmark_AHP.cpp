#include "landmark_AHP.h"

#include "state_homogeneous_3D.h"
#include "factory.h"
#include "yaml/yaml_conversion.h"

namespace wolf {

/* Landmark - Anchored Homogeneous Point*/
LandmarkAHP::LandmarkAHP(Eigen::Vector4s _position_homogeneous,
                         FrameBase* _anchor_frame,
                         SensorBase* _anchor_sensor,
                         cv::Mat _2D_descriptor) :
    LandmarkBase(LANDMARK_AHP, new StateHomogeneous3D(_position_homogeneous)),
    cv_descriptor_(_2D_descriptor.clone()),
    anchor_frame_(_anchor_frame),
    anchor_sensor_(_anchor_sensor)
{
//    std::cout << "LandmarkAHP p_ptr" << p_ptr_->getVector().transpose() << std::endl;
    setType("AHP");
}

LandmarkAHP::~LandmarkAHP()
{
    //
}


wolf::LandmarkBase* LandmarkAHP::create(const YAML::Node& _node)
{
    Eigen::VectorXs pos_homog = _node["position"].as<Eigen::VectorXs>();
    cv::Mat desc(_node["descriptor"].as<std::vector<unsigned int> >());
    return new LandmarkAHP(pos_homog, nullptr, nullptr, desc);
}

// Register landmark creator
namespace
{
const bool registered_lmk_ahp = LandmarkFactory::get().registerCreator("AHP", LandmarkAHP::create);
}

} // namespace wolf
