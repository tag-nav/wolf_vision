#include "base/landmark/landmark_AHP.h"

#include "base/state_homogeneous_3D.h"
#include "base/factory.h"
#include "base/yaml/yaml_conversion.h"

namespace wolf {

/* Landmark - Anchored Homogeneous Point*/
LandmarkAHP::LandmarkAHP(Eigen::Vector4s _position_homogeneous,
                         FrameBasePtr _anchor_frame,
                         SensorBasePtr _anchor_sensor,
                         cv::Mat _2D_descriptor) :
    LandmarkBase("AHP", std::make_shared<StateHomogeneous3D>(_position_homogeneous)),
    cv_descriptor_(_2D_descriptor.clone()),
    anchor_frame_(_anchor_frame),
    anchor_sensor_(_anchor_sensor)
{
}

LandmarkAHP::~LandmarkAHP()
{
    //
}

YAML::Node LandmarkAHP::saveToYaml() const
{
    // First base things
    YAML::Node node = LandmarkBase::saveToYaml();

    // Then add specific things
    std::vector<int> v;
    LandmarkAHP::cv_descriptor_.copyTo(v);
    node["descriptor"] = v;
    return node;
}

Eigen::Vector3s LandmarkAHP::getPointInAnchorSensor() const
{
    Eigen::Map<const Eigen::Vector4s> hmg_point(getPPtr()->getState().data());
    return hmg_point.head<3>()/hmg_point(3);
}

Eigen::Vector3s LandmarkAHP::point() const
{
    using namespace Eigen;
    Transform<Scalar,3,Affine> T_w_r
        = Translation<Scalar,3>(getAnchorFrame()->getPPtr()->getState())
        * Quaternions(getAnchorFrame()->getOPtr()->getState().data());
    Transform<Scalar,3,Affine> T_r_c
        = Translation<Scalar,3>(getAnchorSensor()->getPPtr()->getState())
        * Quaternions(getAnchorSensor()->getOPtr()->getState().data());
    Vector4s point_hmg_c = getPPtr()->getState();
    Vector4s point_hmg = T_w_r * T_r_c * point_hmg_c;
    return point_hmg.head<3>()/point_hmg(3);
}

LandmarkBasePtr LandmarkAHP::create(const YAML::Node& _node)
{
    unsigned int        id          = _node["id"]           .as< unsigned int     >();
    Eigen::VectorXs     pos_homog   = _node["position"]     .as< Eigen::VectorXs  >();
    std::vector<int>    v           = _node["descriptor"]   .as< std::vector<int> >();
    cv::Mat desc(v);

    LandmarkBasePtr lmk = std::make_shared<LandmarkAHP>(pos_homog, nullptr, nullptr, desc);
    lmk->setId(id);
    return lmk;
}

// Register landmark creator
namespace
{
const bool WOLF_UNUSED registered_lmk_ahp = LandmarkFactory::get().registerCreator("AHP", LandmarkAHP::create);
}

} // namespace wolf
