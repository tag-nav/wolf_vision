#include "capture_twist_2D.h"

CaptureTwist2D::CaptureTwist2D(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorTwist2D* _sensor_ptr) :
        CaptureRelative(_init_ts, _sensor_ptr)
{
    //
}

CaptureTwist2D::CaptureTwist2D(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorTwist2D* _sensor_ptr, const Eigen::Vector3s& _data) :
        CaptureRelative(_init_ts, _sensor_ptr, _data)
{
    data_covariance_ = Eigen::Matrix3s::Zero();
    data_covariance_(0, 0) = std::max(1e-10, _sensor_ptr->getLinealNoise());
    data_covariance_(1, 1) = std::max(1e-10, _sensor_ptr->getLinealNoise());
    data_covariance_(2, 2) = std::max(1e-10, _sensor_ptr->getAngularNoise());
//  std::cout << data_covariance_ << std::endl;
}

CaptureTwist2D::CaptureTwist2D(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorTwist2D* _sensor_ptr, const Eigen::Vector3s& _data, const Eigen::Matrix3s& _data_covariance) :
        CaptureRelative(_init_ts, _sensor_ptr, _data, _data_covariance)
{
    //
}

CaptureTwist2D::~CaptureTwist2D()
{
    //std::cout << "Destroying GPS fix capture...\n";
}

inline void CaptureTwist2D::processCapture()
{
    // ADD FEATURE
    addFeature(new FeatureTwist2D(data_, data_covariance_));

    // ADD CONSTRAINT
    addConstraints();
}

Eigen::VectorXs CaptureTwist2D::computePrior(const TimeStamp& _now) const
{
    assert(up_node_ptr_ != nullptr && "This Capture is not linked to any frame");

    WolfScalar dt = (_now - this->time_stamp_).get();
    Eigen::Vector3s disp = dt * data_;

    if (getFramePtr()->getOPtr()->getStateType() == ST_COMPLEX_ANGLE)
    {
        Eigen::Vector4s prior;
        Eigen::Map<Eigen::Vector4s> initial_pose(getFramePtr()->getPPtr()->getPtr());
        //std::cout << "initial_pose: " << initial_pose.transpose() << std::endl;
        prior(0) = initial_pose(0) + disp(0) * initial_pose(2) - disp(1) * initial_pose(3);
        prior(1) = initial_pose(1) + disp(0) * initial_pose(3) + disp(1) * initial_pose(2);
        prior(2) = initial_pose(2) * cos(disp(2)) - initial_pose(3) * sin(disp(2));
        prior(3) = initial_pose(2) * sin(disp(2)) + initial_pose(3) * cos(disp(2));
        //std::cout << "data_: " << data_.transpose() << std::endl;
        //std::cout << "prior: " << prior.transpose() << std::endl;

        return prior;
    }
    else
    {
        Eigen::Vector3s prior;
        Eigen::Map<Eigen::Vector3s> initial_pose(getFramePtr()->getPPtr()->getPtr());
        //std::cout << "initial_pose: " << initial_pose.transpose() << std::endl;
        prior(0) = initial_pose(0) + disp(0) * cos(initial_pose(2)) - disp(1) * sin(initial_pose(2));
        prior(1) = initial_pose(1) + disp(0) * sin(initial_pose(2)) + disp(1) * cos(initial_pose(2));
        prior(2) = initial_pose(2) + disp(2);
        //std::cout << "data_: " << data_.transpose() << std::endl;
        //std::cout << "prior: " << prior.transpose() << std::endl;

        return prior;
    }

}

void CaptureTwist2D::addConstraints()
{
    assert(getFramePtr()->getNextFrame() != nullptr && "Trying to add odometry constraint in the last frame (there is no next frame)");

    getFramePtr()->getNextFrame()

    if (getFramePtr()->getOPtr()->getStateType() == ST_COMPLEX_ANGLE)
    {
        getFeatureListPtr()->front()->addConstraint(new ConstraintOdom2DComplexAngle(getFeatureListPtr()->front(),
                                                                                     getFramePtr()->getPPtr(),
                                                                                     getFramePtr()->getOPtr(),
                                                                                     getFramePtr()->getNextFrame()->getPPtr(),
                                                                                     getFramePtr()->getNextFrame()->getOPtr()));
    }
    else
    {
        getFeatureListPtr()->front()->addConstraint(new ConstraintOdom2DTheta(getFeatureListPtr()->front(),
                                                                              getFramePtr()->getPPtr(),
                                                                              getFramePtr()->getOPtr(),
                                                                              getFramePtr()->getNextFrame()->getPPtr(),
                                                                              getFramePtr()->getNextFrame()->getOPtr()));
    }
}

void CaptureTwist2D::integrateCapture(CaptureRelative* _new_capture)
{
    assert(dynamic_cast<CaptureOdom2D*>(_new_capture) && "Trying to integrate with a CaptureOdom2D a CaptureRelativePtr which is not CaptureOdom2D");

    //std::cout << "Integrate odoms: " << std::endl << data_.transpose() << std::endl << _new_capture->getData().transpose() << std::endl;
    data_(0) += (_new_capture->getData()(0) * cos(data_(2)) - _new_capture->getData()(1) * sin(data_(2)));
    data_(1) += (_new_capture->getData()(0) * sin(data_(2)) + _new_capture->getData()(1) * cos(data_(2)));
    data_(2) += _new_capture->getData()(2);

    // TODO Jacobians!
    data_covariance_ += _new_capture->getDataCovariance();

    final_time_stamp_ = _new_capture->final_time_stamp_;

    getFeatureListPtr()->front()->setMeasurement(data_);
    getFeatureListPtr()->front()->setMeasurementCovariance(data_covariance_);
    //std::cout << "Integrated odoms: " << std::endl << data_.transpose() << std::endl << data_covariance_ << std::endl;
}

CaptureOdom2D* CaptureTwist2D::interpolateCapture(const TimeStamp& _ts)
{
    WolfScalar ratio = (_ts.get() - initial_time_stamp_.get()) / (final_time_stamp_.get() - initial_time_stamp_.get());

    // Second part
    CaptureOdom2D* second_odom_ptr = new CaptureOdom2D(_ts, final_time_stamp_, sensor_ptr_, data_ * (1-ratio), data_covariance_ * (1-ratio));

    // First part (stored in this)
    data_ *= ratio;
    data_covariance_*= ratio;
    final_time_stamp_ = _ts;

    return second_odom_ptr;
}

//void CaptureTwist2D::printSelf(unsigned int _ntabs, std::ostream & _ost) const
//{
//    NodeLinked::printSelf(_ntabs, _ost);
//    //printTabs(_ntabs);
//    //_ost << "\tSensor pose : ( " << sensor_ptr_->pose().x().transpose() << " )" << std::endl;
//    //printNTabs(_ntabs);
//    //_ost << "\tSensor intrinsic : ( " << sensor_ptr_->intrinsic().transpose() << " )" << std::endl;
//}
