#include "vision/landmark/landmark_HP.h"

#include "core/state_block/state_homogeneous_3d.h"
#include "core/common/factory.h"
#include "core/yaml/yaml_conversion.h"

namespace wolf {

/* Landmark - Homogeneous Point*/
LandmarkHP::LandmarkHP(Eigen::Vector4d _position_homogeneous,
						 SensorBasePtr _sensor,
                         cv::Mat _2d_descriptor) :
    LandmarkBase("HP", std::make_shared<StateHomogeneous3d>(_position_homogeneous)),
    cv_descriptor_(_2d_descriptor.clone())
{
}

LandmarkHP::~LandmarkHP()
{
    //
}

YAML::Node LandmarkHP::saveToYaml() const
{
    // First base things
    YAML::Node node = LandmarkBase::saveToYaml();

    // Then add specific things
    std::vector<int> v;
    LandmarkHP::cv_descriptor_.copyTo(v);
    node["descriptor"] = v;
    return node;
}

Eigen::Vector3d LandmarkHP::point() const
{
    using namespace Eigen;

    Vector4d point_hmg = getP()->getState();

    return point_hmg.head<3>()/point_hmg(3);
}

LandmarkBasePtr LandmarkHP::create(const YAML::Node& _node)
{
    unsigned int        id          = _node["id"]           .as< unsigned int     >();
    Eigen::VectorXd     pos_homog   = _node["position"]     .as< Eigen::VectorXd  >();
    std::vector<int>    v           = _node["descriptor"]   .as< std::vector<int> >();
    cv::Mat desc(v);

    LandmarkBasePtr lmk = std::make_shared<LandmarkHP>(pos_homog, nullptr, desc);
    lmk->setId(id);
    return lmk;
}

// Register landmark creator
namespace
{
const bool WOLF_UNUSED registered_lmk_hp = LandmarkFactory::get().registerCreator("HP", LandmarkHP::create);
}

} // namespace wolf
