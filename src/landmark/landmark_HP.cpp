#include "vision/landmark/landmark_HP.h"

#include "core/state_block/state_homogeneous_3D.h"
#include "core/common/factory.h"
#include "core/yaml/yaml_conversion.h"

namespace wolf {

/* Landmark - Homogeneous Point*/
LandmarkHP::LandmarkHP(Eigen::Vector4s _position_homogeneous,
						 SensorBasePtr _sensor,
                         cv::Mat _2D_descriptor) :
    LandmarkBase("HP", std::make_shared<StateHomogeneous3D>(_position_homogeneous)),
	sensor_(_sensor),
    cv_descriptor_(_2D_descriptor.clone())
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

Eigen::Vector3s LandmarkHP::point() const
{
    using namespace Eigen;

    /* TODO: done when creating the landmark
    Transform<Scalar,3,Affine> T_w_r
        = Translation<Scalar,3>(getAnchorFrame()->getP()->getState())
        * Quaternions(getAnchorFrame()->getO()->getState().data());
    Transform<Scalar,3,Affine> T_r_c
        = Translation<Scalar,3>(getAnchorSensor()->getP()->getState())
        * Quaternions(getAnchorSensor()->getO()->getState().data());
    */
    //Vector4s point_hmg_c = getP()->getState();
    //Vector4s point_hmg = T_w_r * T_r_c * point_hmg_c;

    Vector4s point_hmg = getP()->getState();

    return point_hmg.head<3>()/point_hmg(3);
}

LandmarkBasePtr LandmarkHP::create(const YAML::Node& _node)
{
    unsigned int        id          = _node["id"]           .as< unsigned int     >();
    Eigen::VectorXs     pos_homog   = _node["position"]     .as< Eigen::VectorXs  >();
    std::vector<int>    v           = _node["descriptor"]   .as< std::vector<int> >();
    cv::Mat desc(v);

    LandmarkBasePtr lmk = std::make_shared<LandmarkHP>(pos_homog, nullptr, desc);
    lmk->setId(id);
    return lmk;
}

// Register landmark creator
namespace
{
const bool WOLF_UNUSED registered_lmk_ahp = LandmarkFactory::get().registerCreator("HP", LandmarkHP::create);
}

} // namespace wolf
