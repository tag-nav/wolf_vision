#include "base/capture/capture_pose.h"

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

void CapturePose::emplaceFeatureAndFactor()
{
    // Emplace feature
    // FeaturePosePtr feature_pose = std::make_shared<FeaturePose>(data_,data_covariance_);
    // addFeature(feature_pose);
    auto feature_pose = FeatureBase::emplace<FeaturePose>(shared_from_this(), data_, data_covariance_);

    std::cout << data_.size() << " ~~ " << data_covariance_.rows() << "x" << data_covariance_.cols() << std::endl;
    // Emplace factor
    if (data_.size() == 3 && data_covariance_.rows() == 3 && data_covariance_.cols() == 3 )
        FactorBase::emplace<FactorPose2D>(feature_pose, feature_pose);
    else if (data_.size() == 7 && data_covariance_.rows() == 6 && data_covariance_.cols() == 6 )
        FactorBase::emplace<FactorPose3D>(feature_pose, feature_pose);
    else
        throw std::runtime_error("Wrong data size in CapturePose. Use 3 for 2D. Use 7 for 3D.");
}

} // namespace wolf
