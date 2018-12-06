
#include "landmark_apriltag.h"
#include "factory.h"
#include "rotations.h"
#include "yaml/yaml_conversion.h"

namespace wolf {

LandmarkApriltag::LandmarkApriltag(Eigen::Vector7s& pose, const int& _tagid, const Scalar& _tag_width) :
	LandmarkBase("APRILTAG", std::make_shared<StateBlock>(pose.head(3)), std::make_shared<StateQuaternion>(pose.tail(4))),
	tag_width_(_tag_width)
{
  	setDescriptor(Eigen::VectorXs::Constant(1,_tagid)); //change tagid to int ? do not use descriptor vector ?
    setId(_tagid);
}

LandmarkApriltag::~LandmarkApriltag()
{
    //
}


Scalar LandmarkApriltag::getTagWidth() const
{
    return tag_width_;
}

int LandmarkApriltag::getTagId() const
{
    return round(descriptor_(0));
}

// LANDMARK SAVE AND LOAD FROM YAML

// static
LandmarkBasePtr LandmarkApriltag::create(const YAML::Node& _lmk_node)
{
    // Parse YAML node with lmk info and data
    unsigned int    id                      = _lmk_node["id"]                   .as<unsigned int>();
    unsigned int    tag_id                  = _lmk_node["tag id"]               .as<unsigned int>();
    Scalar          tag_width               = _lmk_node["tag width"]            .as<Scalar>();
    Eigen::Vector3s pos                     = _lmk_node["position"]             .as<Eigen::Vector3s>();
    bool            pos_fixed               = _lmk_node["position fixed"]       .as<bool>();
    Eigen::Vector3s ori          = M_TORAD * (_lmk_node["orientation"]          .as<Eigen::Vector3s>() );
    bool            ori_fixed               = _lmk_node["orientation fixed"]    .as<bool>();

    Eigen::Matrix3s       R = matrixRollPitchYaw(ori(0), ori(1), ori(2));
    Eigen::Quaternions quat = R2q(R);
    Eigen::Vector7s pose; pose << pos, quat.coeffs();


    // Create a new landmark
    LandmarkApriltagPtr lmk_ptr = std::make_shared<LandmarkApriltag>(pose, tag_id, tag_width);
    lmk_ptr->setId(id);
    lmk_ptr->getPPtr()->setFixed(pos_fixed);
    lmk_ptr->getOPtr()->setFixed(ori_fixed);

    return lmk_ptr;

}

YAML::Node LandmarkApriltag::saveToYaml() const
{
    // First base things
    YAML::Node node = LandmarkBase::saveToYaml();

//    // Then add specific things
//    node["first_id"]       = first_id_;
//    node["first_defined"]  = first_defined_;
//    node["last_defined"]   = last_defined_;
//    node["classification"] = (int)classification_;
//
//    int npoints = point_state_ptr_vector_.size();
//
//    for (int i = 0; i < npoints; i++)
//    {
//        node["points"].push_back(point_state_ptr_vector_[i]->getState());
//    }

    return node;
}


// Register landmark creator
namespace
{
const bool WOLF_UNUSED registered_lmk_apriltag = LandmarkFactory::get().registerCreator("APRILTAG", LandmarkApriltag::create);
}


} // namespace wolf
