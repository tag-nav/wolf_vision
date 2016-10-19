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

void CaptureFix::process()
{
	// EXTRACT AND ADD FEATURES
    addFeature(new FeatureFix(data_,data_covariance_));
    //std::cout << "FeatureFix extracted " << std::endl;

    // ADD CONSTRAINT
    if (data_.size() == 3 && data_covariance_.rows() == 3 && data_covariance_.cols() == 3 )
        getFeatureListPtr()->front()->addConstraint(new ConstraintFix(getFeatureListPtr()->front()));
    else if (data_.size() == 7 && data_covariance_.rows() == 6 && data_covariance_.cols() == 6 )
        getFeatureListPtr()->front()->addConstraint(new ConstraintFix3D(getFeatureListPtr()->front()));
    else
        throw std::runtime_error("wrong data size in capture fix"); // TODO
    //std::cout << "ConstraintFix added " << std::endl;
}


} // namespace wolf
