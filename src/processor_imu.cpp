#include "processor_imu.h"

namespace wolf {

ProcessorIMU::ProcessorIMU(ProcessorIMUParamsPtr _params) :
        ProcessorMotion("IMU", 10, 10, 9, 6, 0.01, 6),
        max_time_span_  (_params ? _params    ->max_time_span   : 1.0  ),
        max_buff_length_(_params ? _params    ->max_buff_length : 10000   ),
        dist_traveled_  (_params ? _params    ->dist_traveled   : 1.0  ),
        angle_turned_   (_params ? _params    ->angle_turned    : 0.2  ),
        voting_active_  (_params ? _params    ->voting_active    : false  )
{
    // Set constant parts of Jacobians
    jacobian_delta_preint_.setIdentity(9,9);                                    // dDp'/dDp, dDv'/dDv, all zeros
    jacobian_delta_.setIdentity(9,9);                                           //
    jacobian_calib_.setZero(9,6);
}

ProcessorIMU::~ProcessorIMU()
{
    //
}

ProcessorBasePtr ProcessorIMU::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sen_ptr)
{
    // cast inputs to the correct type
    std::shared_ptr<ProcessorIMUParams> prc_imu_params = std::static_pointer_cast<ProcessorIMUParams>(_params);

    ProcessorIMUPtr prc_ptr = std::make_shared<ProcessorIMU>(prc_imu_params);
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

bool ProcessorIMU::voteForKeyFrame()
{
    if(!voting_active_)
        return false;
    // time span
    if (getBuffer().get().back().ts_ - getBuffer().get().front().ts_ > max_time_span_)
    {
        return true;
    }
    // buffer length
    if (getBuffer().get().size() > max_buff_length_)
    {
        return true;
    }
    /*// angle turned
    Scalar angle = 2.0 * acos(delta_integrated_(6));
    if (angle > angle_turned_)
    {
        WOLF_DEBUG( "PM: vote: angle turned" );
        return true;
    }*/
    //WOLF_DEBUG( "PM: do not vote" );
    return false;
}

Motion ProcessorIMU::interpolate(const Motion& _motion_ref, Motion& _motion_second, TimeStamp& _ts)
{
    /* Note: See extensive documentation in ProcessorMotion::interpolate().
     *
     * Interpolate between motion_ref and motion, as in:
     *
     *    motion_ref ------ ts_ ------ motion_sec
     *                    return
     *
     * and return the motion at the given time_stamp ts_.
     *
     * DATA:
     * Data receives no change
     *
     * DELTA:
     * The delta's position and velocity receive linear interpolation:
     *    p_ret = (ts - t_ref) / dt * (p - p_ref)
     *    v_ret = (ts - t_ref) / dt * (v - v_ref)
     *
     * The delta's quaternion receives a slerp interpolation
     *    q_ret = q_ref.slerp( (ts - t_ref) / dt , q);
     *
     * DELTA_INTEGR:
     * The interpolated delta integral is just the reference integral plus the interpolated delta
     *
     * The second integral does not change
     *
     * Covariances receive linear interpolation
     *    Q_ret = (ts - t_ref) / dt * Q_sec
     */
    // resolve out-of-bounds time stamp as if the time stamp was exactly on the bounds
    if (_ts <= _motion_ref.ts_)    // behave as if _ts == _motion_ref.ts_
    {
        // return null motion. Second stays the same.
        Motion motion_int     ( _motion_ref );
        motion_int.data_      = _motion_second.data_;
        motion_int.data_cov_  = _motion_second.data_cov_;
        motion_int.delta_     = deltaZero();
        motion_int.delta_cov_ . setZero();
        return motion_int;
    }
    if (_motion_second.ts_ <= _ts)    // behave as if _ts == _motion_second.ts_
    {
        // return original second motion. Second motion becomes null motion
        Motion motion_int         ( _motion_second );
        _motion_second.delta_     = deltaZero();
        _motion_second.delta_cov_ . setZero();
        return motion_int;
    }
    assert(_motion_ref.ts_ <= _ts && "Interpolation time stamp out of bounds");
    assert(_ts <= _motion_second.ts_ && "Interpolation time stamp out of bounds");

    assert(_motion_ref.delta_.size() == delta_size_ && "Wrong delta size");
    assert(_motion_ref.delta_cov_.cols() == delta_cov_size_ && "Wrong delta cov size");
    assert(_motion_ref.delta_cov_.rows() == delta_cov_size_ && "Wrong delta cov size");
    assert(_motion_ref.delta_integr_.size() == delta_size_ && "Wrong delta size");
    assert(_motion_ref.delta_integr_cov_.cols() == delta_cov_size_ && "Wrong delta cov size");
    assert(_motion_ref.delta_integr_cov_.rows() == delta_cov_size_ && "Wrong delta cov size");

    assert(_motion_second.delta_.size() == delta_size_ && "Wrong delta size");
    assert(_motion_second.delta_cov_.cols() == delta_cov_size_ && "Wrong delta cov size");
    assert(_motion_second.delta_cov_.rows() == delta_cov_size_ && "Wrong delta cov size");
    assert(_motion_second.delta_integr_.size() == delta_size_ && "Wrong delta size");
    assert(_motion_second.delta_integr_cov_.cols() == delta_cov_size_ && "Wrong delta cov size");
    assert(_motion_second.delta_integr_cov_.rows() == delta_cov_size_ && "Wrong delta cov size");

    // reference
    TimeStamp t_ref = _motion_ref.ts_;

    // second
    TimeStamp t_sec = _motion_second.ts_;
    Map<VectorXs>    motion_sec_dp  (_motion_second.delta_.data() + 0, 3);
    Map<Quaternions> motion_sec_dq  (_motion_second.delta_.data() + 3   );
    Map<VectorXs>    motion_sec_dv  (_motion_second.delta_.data() + 7, 3);

    // interpolated
    Motion motion_int = motionZero(_ts);

    // Jacobians for covariance propagation
    MatrixXs J_ref(delta_cov_size_, delta_cov_size_);
    MatrixXs J_int(delta_cov_size_, delta_cov_size_);

    // interpolation factor
    Scalar dt1    = _ts - t_ref;
    Scalar dt2    = t_sec - _ts;
    Scalar tau    = dt1 / (t_sec - t_ref); // interpolation factor (0 to 1)
    Scalar tau_sq = tau * tau;

    // copy data
    motion_int.data_      = _motion_second.data_;
    motion_int.data_cov_  = _motion_second.data_cov_;

    // interpolate delta
    motion_int.ts_        = _ts;
    Map<VectorXs>    motion_int_dp  (motion_int.delta_.data() + 0, 3);
    Map<Quaternions> motion_int_dq  (motion_int.delta_.data() + 3   );
    Map<VectorXs>    motion_int_dv  (motion_int.delta_.data() + 7, 3);
    motion_int_dp         = tau_sq * motion_sec_dp; // FIXME: delta_p not correctly interpolated
    motion_int_dv         = tau    * motion_sec_dv;
    motion_int_dq         = Quaternions::Identity().slerp(tau, motion_sec_dq);
    motion_int.delta_cov_ = tau    * _motion_second.delta_cov_;

    // integrate
    deltaPlusDelta(_motion_ref.delta_integr_, motion_int.delta_, dt1, motion_int.delta_integr_, J_ref, J_int);
    motion_int.delta_integr_cov_ = J_ref * _motion_ref.delta_integr_cov_ * J_ref.transpose()
                                 + J_int * _motion_second.delta_cov_     * J_int.transpose();

    // update second delta ( in place update )
    motion_sec_dp = motion_int_dq.conjugate() * (motion_sec_dp - motion_int_dp - motion_int_dv * dt2);
    motion_sec_dv = motion_int_dq.conjugate() * (motion_sec_dv - motion_int_dv);
    motion_sec_dq = motion_int_dq.conjugate() *  motion_sec_dq;
    _motion_second.delta_cov_ *= (1 - tau); // easy interpolation // TODO check for correctness
    //Dp            = Dp; // trivial, just leave the code commented
    //Dq            = Dq; // trivial, just leave the code commented
    //_motion.delta_integr_cov_ = _motion.delta_integr_cov_; // trivial, just leave the code commented

    return motion_int;
}

CaptureMotionPtr ProcessorIMU::createCapture(const TimeStamp& _ts,
                                             const SensorBasePtr& _sensor,
                                             const VectorXs& _data,
                                             const MatrixXs& _data_cov,
                                             const FrameBasePtr& _frame_origin)
{
    CaptureIMUPtr capture_imu = std::make_shared<CaptureIMU>(_ts,
                                                             _sensor,
                                                             _data,
                                                             _data_cov,
                                                             _frame_origin);
    return capture_imu;
}

FeatureBasePtr ProcessorIMU::createFeature(CaptureMotionPtr _capture_motion)
{
    FeatureIMUPtr key_feature_ptr = std::make_shared<FeatureIMU>(
            _capture_motion->getBuffer().get().back().delta_integr_,
            _capture_motion->getBuffer().get().back().delta_integr_cov_,
            _capture_motion->getBuffer().getCalibrationPreint(),
            _capture_motion->getBuffer().get().back().jacobian_calib_);
    return key_feature_ptr;
}

ConstraintBasePtr ProcessorIMU::emplaceConstraint(FeatureBasePtr _feature_motion, CaptureBasePtr _capture_origin)
{
    CaptureIMUPtr cap_imu = std::static_pointer_cast<CaptureIMU>(_capture_origin);
    FeatureIMUPtr ftr_imu = std::static_pointer_cast<FeatureIMU>(_feature_motion);
    ConstraintIMUPtr ctr_imu = std::make_shared<ConstraintIMU>(ftr_imu, cap_imu, shared_from_this());

    // link ot wolf tree
    _feature_motion->addConstraint(ctr_imu);
    _capture_origin->addConstrainedBy(ctr_imu);
    _capture_origin->getFramePtr()->addConstrainedBy(ctr_imu);

    return ctr_imu;
}

void ProcessorIMU::computeCurrentDelta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov,
                                       const Eigen::VectorXs& _calib, const Scalar _dt, Eigen::VectorXs& _delta,
                                       Eigen::MatrixXs& _delta_cov, Eigen::MatrixXs& _jacobian_calib)
{
    assert(_data.size() == data_size_ && "Wrong data size!");

    using namespace Eigen;
    Matrix<Scalar, 9, 6> jac_data;

    /*
     * We have the following computing pipeline:
     *     Input: data, calib, dt
     *     Output: delta, delta_cov, jac_calib
     *
     * We do:
     *     body         = data - calib
     *     delta        = body2delta(body, dt) --> jac_body
     *     jac_data     = jac_body
     *     jac_calib    = - jac_body
     *     delta_cov  <-- propagate data_cov using jac_data
     *
     * where body2delta(.) produces a delta from body=(a,w) as follows:
     *     dp = 1/2 * a * dt^2
     *     dq = exp(w * dt)
     *     dv = a * dt
     */

    // create delta
    imu::body2delta(_data - _calib, _dt, _delta, jac_data); // Jacobians tested in imu_tools

    // compute delta_cov
    _delta_cov = jac_data * _data_cov * jac_data.transpose();

    // compute jacobian_calib
    _jacobian_calib = -jac_data;

}

void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta_preint, const Eigen::VectorXs& _delta, const Scalar _dt,
                                  Eigen::VectorXs& _delta_preint_plus_delta)
{
    /* MATHS according to Sola-16
     * Dp' = Dp + Dv*dt + 1/2*Dq*(a-a_b)*dt^2    = Dp + Dv*dt + Dq*dp   if  dp = 1/2*(a-a_b)*dt^2
     * Dv' = Dv + Dq*(a-a_b)*dt                  = Dv + Dq*dv           if  dv = (a-a_b)*dt
     * Dq' = Dq * exp((w-w_b)*dt)                = Dq * dq              if  dq = exp((w-w_b)*dt)
     *
     * where (dp, dq, dv) need to be computed in data2delta(), and Dq*dx =is_equivalent_to= Dq*dx*Dq'.
     */
    _delta_preint_plus_delta = imu::compose(_delta_preint, _delta, _dt);
}

void ProcessorIMU::statePlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _dt,
                                  Eigen::VectorXs& _x_plus_delta)
{
    //    assert(_x.size() == 10 && "Wrong _x vector size");
    //    assert(_delta.size() == 10 && "Wrong _delta vector size");
    //    assert(_x_plus_delta.size() == 10 && "Wrong _x_plus_delta vector size");
    assert(_dt >= 0 && "Time interval _Dt is negative!");
    VectorXs x(_x.head(10));
    VectorXs x_plus_delta(10);
    x_plus_delta = imu::composeOverState(x, _delta, _dt);
    _x_plus_delta.head(10) = x_plus_delta;
}

void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta_preint, const Eigen::VectorXs& _delta, const Scalar _dt,
                                  Eigen::VectorXs& _delta_preint_plus_delta, Eigen::MatrixXs& _jacobian_delta_preint,
                                  Eigen::MatrixXs& _jacobian_delta)
{
    /*
     * Expression of the delta integration step, D' = D (+) d:
     *
     *     Dp' = Dp + Dv*dt + Dq*dp
     *     Dv' = Dv + Dq*dv
     *     Dq' = Dq * dq
     *
     * Jacobians for covariance propagation.
     *
     * a. With respect to Delta, gives _jacobian_delta_preint = D_D as:
     *
     *   D_D = [ I    -DR*skew(dp)   I*dt
     *           0     dR.tr          0
     *           0    -DR*skew(dv)    I  ] // See Sola-16
     *
     * b. With respect to delta, gives _jacobian_delta = D_d as:
     *
     *   D_d = [ DR   0    0
     *           0    I    0
     *           0    0    DR ] // See Sola-16
     *
     * Note: covariance propagation, i.e.,  P+ = D_D * P * D_D' + D_d * M * D_d', is done in ProcessorMotion.
     */
    imu::compose(_delta_preint, _delta, _dt, _delta_preint_plus_delta, _jacobian_delta_preint, _jacobian_delta); // jacobians tested in imu_tools
}

} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"

namespace wolf
{
WOLF_REGISTER_PROCESSOR("IMU", ProcessorIMU)
}
