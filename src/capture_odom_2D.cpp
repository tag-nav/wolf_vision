#include "capture_odom_2D.h"
#include <cmath>

namespace wolf {

CaptureOdom2D::CaptureOdom2D(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr,
                             const Eigen::Vector3s& _data) :
        CaptureMotion(_init_ts, _final_ts, _sensor_ptr, _data)
{
    setType("ODOM 2D");
    data_covariance_ = Eigen::Matrix3s::Zero();
    data_covariance_(0, 0) = std::max((Scalar)1e-10, _data(0) * ((SensorOdom2D*)_sensor_ptr)->getDispVarToDispNoiseFactor());
    data_covariance_(1, 1) = std::max((Scalar)1e-10, _data(1) * ((SensorOdom2D*)_sensor_ptr)->getDispVarToDispNoiseFactor());
    data_covariance_(2, 2) = std::max((Scalar)1e-10, _data(2) * ((SensorOdom2D*)_sensor_ptr)->getRotVarToRotNoiseFactor());
//  std::cout << data_covariance_ << std::endl;
}

CaptureOdom2D::CaptureOdom2D(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr,
                             const Eigen::Vector3s& _data, const Eigen::Matrix3s& _data_covariance) :
        CaptureMotion(_init_ts, _final_ts, _sensor_ptr, _data, _data_covariance)
{
    setType("ODOM 2D");
}

CaptureOdom2D::~CaptureOdom2D()
{
    //std::cout << "Destroying CaptureOdom2D capture...\n";
}

inline void CaptureOdom2D::process()
{
    // ADD FEATURE
    addFeature(new FeatureOdom2D(data_, data_covariance_));

    // ADD CONSTRAINT
    addConstraints();
}

Eigen::VectorXs CaptureOdom2D::computeFramePose(const TimeStamp& _now) const
{
    //std::cout << "compute frame pose in stamp" << _now.getNanoSeconds() << std::endl;

    assert(getFramePtr() != nullptr && "This Capture is not linked to any frame");
    assert(getFramePtr()->getPreviousFrame() != nullptr && "There is not previous frame");

    //std::cout << "previous frame: " << getFramePtr()->getPreviousFrame()->nodeId() << std::endl;

    Eigen::Vector3s prior = getFramePtr()->getPreviousFrame()->getState();
    //std::cout << "previous pose: " << prior.transpose() << std::endl;
    //std::cout << "previous pose: " << getFramePtr()->getPreviousFrame()->getPPtr()->getVector().transpose() << " " << getFramePtr()->getPreviousFrame()->getOPtr()->getVector().transpose() << std::endl;
    prior(0) += data_(0) * cos(prior(2)) - data_(1) * sin(prior(2));
    prior(1) += data_(0) * sin(prior(2)) + data_(1) * cos(prior(2));
    prior(2) += data_(2);
    //std::cout << "data_: " << data_.transpose() << std::endl;
    //std::cout << "prior: " << prior.transpose() << std::endl;

    return prior;

}

void CaptureOdom2D::addConstraints()
{
    assert(getFramePtr()->getPreviousFrame() != nullptr && "Trying to add odometry constraint in the first frame (there is no previous frame)");

    getFeatureListPtr()->front()->addConstraint(new ConstraintOdom2DAnalytic(getFeatureListPtr()->front(),
			  	  	  	  	  	  	  	  	  	  	  	  	  	                 getFramePtr()->getPreviousFrame()));
}

void CaptureOdom2D::integrateCapture(CaptureMotion* _new_capture)
{
    assert(dynamic_cast<CaptureOdom2D*>(_new_capture) && "Trying to integrate with a CaptureOdom2D a CaptureRelativePtr which is not CaptureOdom2D");

    //std::cout << "Integrate odoms: " << std::endl << data_.transpose() << std::endl << _new_capture->getData().transpose() << std::endl;
    // turn/2 + straight + turn/2
    data_(2) += _new_capture->getData()(2)/2;
    data_(0) += _new_capture->getData()(0) * cos(data_(2)) - _new_capture->getData()(1) * sin(data_(2));
    data_(1) += _new_capture->getData()(0) * sin(data_(2)) + _new_capture->getData()(1) * cos(data_(2));
    data_(2) += _new_capture->getData()(2)/2;

    // Covariance propagation
    Eigen::Matrix3s Jx = Eigen::Matrix3s::Identity();
    Jx(0,2) = -_new_capture->getData()(0)*sin(data_(2)+_new_capture->getData()(2)/2);
    Jx(1,2) = _new_capture->getData()(0)*cos(data_(2)+_new_capture->getData()(2)/2);

    Eigen::Matrix3s Ju = Eigen::Matrix3s::Zero();
    Ju(0,0) = cos(data_(2)+_new_capture->getData()(2)/2);
    Ju(1,0) = sin(data_(2)+_new_capture->getData()(2)/2);
    Ju(0,2) = -0.5*_new_capture->getData()(0)*cos(data_(2)+_new_capture->getData()(2)/2);
    Ju(1,2) = 0.5*_new_capture->getData()(0)*cos(data_(2)+_new_capture->getData()(2)/2);
    Ju(2,2) = 1;

    data_covariance_ = Jx*data_covariance_*Jx.transpose() + Ju*_new_capture->getDataCovariance()*Ju.transpose();

    this->final_time_stamp_ = _new_capture->getFinalTimeStamp();

    getFeatureListPtr()->front()->setMeasurement(data_);
    getFeatureListPtr()->front()->setMeasurementCovariance(data_covariance_);
    //std::cout << "Integrated odoms: " << std::endl << data_.transpose() << std::endl << data_covariance_ << std::endl;
}

CaptureOdom2D* CaptureOdom2D::interpolateCapture(const TimeStamp& _ts)
{
    Scalar ratio = (_ts.get() - this->time_stamp_.get()) / (this->final_time_stamp_.get() - this->time_stamp_.get());

    // Second part
    CaptureOdom2D* second_odom_ptr = new CaptureOdom2D(_ts, final_time_stamp_, sensor_ptr_, data_ * (1-ratio), data_covariance_ * (1-ratio));

    // First part (stored in this)
    data_ *= ratio;
    data_covariance_*= ratio;
    final_time_stamp_ = _ts;

    return second_odom_ptr;
}

} // namespace wolf
