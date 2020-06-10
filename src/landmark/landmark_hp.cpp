#include "vision/landmark/landmark_hp.h"

#include "core/state_block/state_homogeneous_3d.h"
#include "core/common/factory.h"
#include "core/yaml/yaml_conversion.h"

namespace wolf {

/* Landmark - Homogeneous Point*/
LandmarkHp::LandmarkHp(Eigen::Vector4d _position_homogeneous,
						 SensorBasePtr _sensor,
                         cv::Mat _2d_descriptor) :
    LandmarkBase("LandmarkHp", std::make_shared<StateHomogeneous3d>(_position_homogeneous)),
    cv_descriptor_(_2d_descriptor.clone())
{
}

LandmarkHp::~LandmarkHp()
{
    //
}

YAML::Node LandmarkHp::saveToYaml() const
{
    // First base things
    YAML::Node node = LandmarkBase::saveToYaml();

    // Then add specific things
    std::vector<int> v;
    LandmarkHp::cv_descriptor_.copyTo(v);
    node["descriptor"] = v;
    return node;
}

Eigen::Vector3d LandmarkHp::point() const
{
    using namespace Eigen;

    Vector4d point_hmg = getP()->getState();

    return point_hmg.head<3>()/point_hmg(3);
}

LandmarkBasePtr LandmarkHp::create(const YAML::Node& _node)
{
    unsigned int        id          = _node["id"]           .as< unsigned int     >();
    Eigen::VectorXd     pos_homog   = _node["position"]     .as< Eigen::VectorXd  >();
    std::vector<int>    v           = _node["descriptor"]   .as< std::vector<int> >();
    cv::Mat desc(v);

    LandmarkBasePtr lmk = std::make_shared<LandmarkHp>(pos_homog, nullptr, desc);
    lmk->setId(id);
    return lmk;
}

// Register landmark creator
namespace
{
const bool WOLF_UNUSED registered_lmk_hp = FactoryLandmark::registerCreator("LandmarkHp", LandmarkHp::create);
}

} // namespace wolf
