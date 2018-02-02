#include "capture_pose.h"

namespace wolf{

CapturePose::CapturePose(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase("POSE", _ts, _sensor_ptr),
	data_(_data),
	data_covariance_(_data_covariance)
{
    //
}

CapturePose::~CapturePose()
{
	//
}

void CapturePose::emplaceFeatureAndConstraint()
{
    // Emplace feature
    FeaturePosePtr feature_pose = std::make_shared<FeaturePose>(data_,data_covariance_);
    addFeature(feature_pose);

    // Emplace constraint
    if (data_.size() == 3 && data_covariance_.rows() == 3 && data_covariance_.cols() == 3 )
        feature_pose->addConstraint(std::make_shared<ConstraintPose2D>(feature_pose));
    else if (data_.size() == 7 && data_covariance_.rows() == 6 && data_covariance_.cols() == 6 )
        feature_pose->addConstraint(std::make_shared<ConstraintPose3D>(feature_pose));
    else
        throw std::runtime_error("Wrong data size in CapturePose. Use 3 for 2D. Use 7 for 3D.");
}


} // namespace wolf
