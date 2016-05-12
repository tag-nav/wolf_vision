#include "capture_fix.h"

namespace wolf{

CaptureFix::CaptureFix(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance) :
	CaptureBase(_ts, _sensor_ptr),
	data_(_data),
	data_covariance_(_data_covariance)
{
    setType("FIX");
    //std::cout << "capture fix constructor " << std::endl;
}

CaptureFix::~CaptureFix()
{
	//
}

void CaptureFix::process()
{
	// EXTRACT AND ADD FEATURES
    addFeature(new FeatureFix(data_,data_covariance_));
    //std::cout << "FeatureFix extracted " << std::endl;

    // ADD CONSTRAINT
    if (data_.size() == 3 && data_covariance_.rows() == 3 && data_covariance_.cols() == 3 )
        getFeatureListPtr()->front()->addConstraint(new ConstraintFix(getFeatureListPtr()->front()));
    else
        throw std::runtime_error("fix constraint not implemented in 3D"); // TODO
    //std::cout << "ConstraintFix added " << std::endl;
}

Eigen::VectorXs CaptureFix::computeFramePose(const TimeStamp& _now) const
{
	return data_;
}

} // namespace wolf
