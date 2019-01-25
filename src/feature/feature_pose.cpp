#include "feature_pose.h"


namespace wolf {

FeaturePose::FeaturePose(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase("POSE", _measurement, _meas_covariance)
{
    //
}

FeaturePose::~FeaturePose()
{
    //
}

} // namespace wolf
