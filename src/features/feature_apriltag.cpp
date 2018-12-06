
#include "feature_apriltag.h"

namespace wolf {

FeatureApriltag::FeatureApriltag(const Eigen::Vector7s & _measurement, const Eigen::Matrix6s & _meas_covariance, const int _tag_id) : //, const apriltag_detection_t & _det) :
    FeatureBase("APRILTAG", _measurement, _meas_covariance), tag_id_(_tag_id) //, detection_(_det)
{
    //_measurement : translation + quaternion
    // meas_cov : a definir
    //std::cout << "feature: "<< _measurement.transpose() << std::endl;
    setTrackId(_tag_id);
}

FeatureApriltag::~FeatureApriltag()
{
    //
}

Scalar FeatureApriltag::getTagId() const
{
    return tag_id_;
}

//const apriltag_detection_t& FeatureApriltag::getDetection() const
//{
//    return detection_;
//}
} // namespace wolf
