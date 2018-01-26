#include "capture_fix.h"

namespace wolf{

CaptureFix::CaptureFix(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase("FIX", _ts, _sensor_ptr),
	data_(_data),
	data_covariance_(_data_covariance)
{
    //
}

CaptureFix::~CaptureFix()
{
	//
}

void CaptureFix::emplaceFeatureAndConstraint()
{
    // Emplace feature
    FeatureFixPtr feature_fix = std::make_shared<FeatureFix>(data_,data_covariance_);
    addFeature(feature_fix);

    // Emplace constraint
    if (data_.size() == 3 && data_covariance_.rows() == 3 && data_covariance_.cols() == 3 )
        feature_fix->addConstraint(std::make_shared<ConstraintPose2D>(feature_fix));
    else if (data_.size() == 7 && data_covariance_.rows() == 6 && data_covariance_.cols() == 6 )
        feature_fix->addConstraint(std::make_shared<ConstraintPose3D>(feature_fix));
    else
        throw std::runtime_error("Wrong data size in CaptureFix. Use 3 for 2D. Use 7 for 3D.");
}


} // namespace wolf
