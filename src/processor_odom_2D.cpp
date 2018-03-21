#include "processor_odom_2D.h"
namespace wolf
{

ProcessorOdom2D::ProcessorOdom2D(const ProcessorParamsOdom2D& _params) :
                ProcessorMotion("ODOM 2D", _params.time_tolerance, 3, 3, 3, 2, 0),
                dist_traveled_th_(_params.dist_traveled_th_),
                theta_traveled_th_(_params.theta_traveled_th_),
                cov_det_th_(_params.cov_det_th_),
                elapsed_time_th_(_params.elapsed_time_th_)
{
    unmeasured_perturbation_cov_ = _params.unmeasured_perturbation_std_ * _params.unmeasured_perturbation_std_ * Matrix3s::Identity();
}

ProcessorOdom2D::~ProcessorOdom2D()
{
}

void ProcessorOdom2D::computeCurrentDelta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov,
                                          const Eigen::VectorXs& _calib, const Scalar _dt, Eigen::VectorXs& _delta,
                                          Eigen::MatrixXs& _delta_cov, Eigen::MatrixXs& _jacobian_calib)
{
    //std::cout << "ProcessorOdom2d::data2delta" << std::endl;
    assert(_data.size() == data_size_ && "Wrong _data vector size");
    assert(_data_cov.rows() == data_size_ && "Wrong _data_cov size");
    assert(_data_cov.cols() == data_size_ && "Wrong _data_cov size");
    // data  is [dtheta, dr]
    // delta is [dx, dy, dtheta]
    // motion model is 1/2 turn + straight + 1/2 turn
    _delta(0) = cos(_data(1) / 2) * _data(0);
    _delta(1) = sin(_data(1) / 2) * _data(0);
    _delta(2) = _data(1);
    // Fill delta covariance
    Eigen::MatrixXs J(delta_cov_size_, data_size_);
    J(0, 0) = cos(_data(1) / 2);
    J(1, 0) = sin(_data(1) / 2);
    J(2, 0) = 0;
    J(0, 1) = -_data(0) * sin(_data(1) / 2) / 2;
    J(1, 1) = _data(0) * cos(_data(1) / 2) / 2;
    J(2, 1) = 1;
    // Since input data is size 2, and delta is size 3, the noise model must be given by:
    // 1. Covariance of the input data:  J*Q*J.tr
    // 2. Fix variance term to be added: var*Id
    _delta_cov = J * _data_cov * J.transpose() + unmeasured_perturbation_cov_;
    //std::cout << "data      :" << _data.transpose() << std::endl;
    //std::cout << "data cov  :" << std::endl << _data_cov << std::endl;
    //std::cout << "delta     :" << delta_.transpose() << std::endl;
    //std::cout << "delta cov :" << std::endl << delta_cov_ << std::endl;
    // jacobian_delta_calib_ not used in this class yet. In any case, set to zero with:
    //    jacobian_delta_calib_.setZero();
}

void ProcessorOdom2D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, const Scalar _Dt2,
                                     Eigen::VectorXs& _delta1_plus_delta2)
{
    // This is just a frame composition in 2D
    //std::cout << "ProcessorOdom2d::deltaPlusDelta" << std::endl;
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");
    _delta1_plus_delta2.head<2>() = _delta1.head<2>() + Eigen::Rotation2Ds(_delta1(2)).matrix() * _delta2.head<2>();
    _delta1_plus_delta2(2) = pi2pi(_delta1(2) + _delta2(2));
}

void ProcessorOdom2D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, const Scalar _Dt2,
                                     Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                     Eigen::MatrixXs& _jacobian2)
{
    //std::cout << "ProcessorOdom2d::deltaPlusDelta jacobians" << std::endl;
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");
    assert(_jacobian1.rows() == delta_cov_size_ && "Wrong _jacobian1 size");
    assert(_jacobian1.cols() == delta_cov_size_ && "Wrong _jacobian1 size");
    assert(_jacobian2.rows() == delta_cov_size_ && "Wrong _jacobian2 size");
    assert(_jacobian2.cols() == delta_cov_size_ && "Wrong _jacobian2 size");
    _delta1_plus_delta2.head<2>() = _delta1.head<2>() + Eigen::Rotation2Ds(_delta1(2)).matrix() * _delta2.head<2>();
    _delta1_plus_delta2(2) = pi2pi(_delta1(2) + _delta2(2));
    // Jac wrt delta_integrated
    _jacobian1 = Eigen::MatrixXs::Identity(delta_cov_size_, delta_cov_size_);
    _jacobian1(0, 2) = -sin(_delta1(2)) * _delta2(0) - cos(_delta1(2)) * _delta2(1);
    _jacobian1(1, 2) = cos(_delta1(2)) * _delta2(0) - sin(_delta1(2)) * _delta2(1);
    // jac wrt delta
    _jacobian2 = Eigen::MatrixXs::Identity(delta_cov_size_, delta_cov_size_);
    _jacobian2.topLeftCorner<2, 2>() = Eigen::Rotation2Ds(_delta1(2)).matrix();
}

void ProcessorOdom2D::statePlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _Dt,
                                     Eigen::VectorXs& _x_plus_delta)
{
    // This is just a frame composition in 2D
    //std::cout << "ProcessorOdom2d::statePlusDelta" << std::endl;
    assert(_x.size() == x_size_ && "Wrong _x vector size");
    assert(_x_plus_delta.size() == x_size_ && "Wrong _x_plus_delta vector size");
    _x_plus_delta.head<2>() = _x.head<2>() + Eigen::Rotation2Ds(_x(2)).matrix() * _delta.head<2>();
    _x_plus_delta(2) = pi2pi(_x(2) + _delta(2));
}

Motion ProcessorOdom2D::interpolate(const Motion& _ref, Motion& _second, TimeStamp& _ts)
{
    // TODO: Implement actual interpolation
    // Implementation: motion ref keeps the same
    //
//    Motion _interpolated(_ref);
//    _interpolated.ts_ = _ts;
//    _interpolated.data_ = Vector3s::Zero();
//    _interpolated.data_cov_ = Matrix3s::Zero();
//    _interpolated.delta_ = deltaZero();
//    _interpolated.delta_cov_ = Eigen::MatrixXs::Zero(delta_size_, delta_size_);
//    _interpolated.delta_integr_ = _ref.delta_integr_;
//    _interpolated.delta_integr_cov_ = _ref.delta_integr_cov_;
//    _interpolated.jacobian_delta_integr_.setIdentity();
//    _interpolated.jacobian_delta_.setZero();
//    _interpolated.jacobian_calib_.setZero();
//    return _interpolated;

    return ProcessorMotion::interpolate(_ref, _second, _ts);

}

bool ProcessorOdom2D::voteForKeyFrame()
{
    // Distance criterion
    if (getBuffer().get().back().delta_integr_.head<2>().norm() > dist_traveled_th_)
    {
        return true;
    }
    if (getBuffer().get().back().delta_integr_.tail<1>().norm() > theta_traveled_th_)
    {
        return true;
    }
    // Uncertainty criterion
    if (getBuffer().get().back().delta_integr_cov_.determinant() > cov_det_th_)
    {
        return true;
    }
    // Time criterion
    if (getBuffer().get().back().ts_.get() - origin_ptr_->getFramePtr()->getTimeStamp().get() > elapsed_time_th_)
    {
        return true;
    }
    return false;
}

CaptureMotionPtr ProcessorOdom2D::createCapture(const TimeStamp& _ts, const SensorBasePtr& _sensor,
                                                const VectorXs& _data, const MatrixXs& _data_cov,
                                                const FrameBasePtr& _frame_origin)
{
    CaptureOdom2DPtr capture_odom = std::make_shared<CaptureOdom2D>(_ts, _sensor, _data, _data_cov, _frame_origin);
    return capture_odom;
}

ConstraintBasePtr ProcessorOdom2D::emplaceConstraint(FeatureBasePtr _feature, CaptureBasePtr _capture_origin)
{
    ConstraintOdom2DPtr ctr_odom = std::make_shared<ConstraintOdom2D>(_feature, _capture_origin->getFramePtr(),
                                                                      shared_from_this());
    _feature->addConstraint(ctr_odom);
    _capture_origin->getFramePtr()->addConstrainedBy(ctr_odom);
    return ctr_odom;
}

FeatureBasePtr ProcessorOdom2D::createFeature(CaptureMotionPtr _capture_motion)
{
    FeatureBasePtr key_feature_ptr = std::make_shared<FeatureBase>(
            "ODOM 2D", _capture_motion->getBuffer().get().back().delta_integr_,
            _capture_motion->getBuffer().get().back().delta_integr_cov_);
    return key_feature_ptr;
}

ProcessorBasePtr ProcessorOdom2D::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr)
{

    ProcessorOdom2DPtr prc_ptr;

    if (_params)
    {
        std::shared_ptr<ProcessorParamsOdom2D> params = std::static_pointer_cast<ProcessorParamsOdom2D>(_params);

        prc_ptr = std::make_shared<ProcessorOdom2D>(*params);
    }
    else
    {
        std::cout << __FILE__ << ":" << __FUNCTION__ << "() : No parameters provided. Using default set." << std::endl;

        prc_ptr = std::make_shared<ProcessorOdom2D>();
    }

    prc_ptr->setName(_unique_name);

    return prc_ptr;
}



}


// Register in the ProcessorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("ODOM 2D", ProcessorOdom2D)
} // namespace wolf
