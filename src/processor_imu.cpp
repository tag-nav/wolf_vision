#include "processor_imu.h"

namespace wolf {

ProcessorIMU::ProcessorIMU(ProcessorIMUParamsPtr _params) :
        ProcessorMotion("IMU", 16, 10, 9, 6, 0.01, 6),
        max_time_span_  (_params ? _params    ->max_time_span   : 1.0  ),
        max_buff_length_(_params ? _params    ->max_buff_length : 10000   ),
        dist_traveled_  (_params ? _params    ->dist_traveled   : 1.0  ),
        angle_turned_   (_params ? _params    ->angle_turned    : 0.2  ),
        voting_active_  (_params ? _params    ->voting_active    : false  ),
        frame_imu_ptr_(nullptr),
        gravity_(wolf::gravity()),
        acc_bias_(&calib_(0)),
        gyro_bias_(&calib_(3)),
        acc_measured_(nullptr),
        gyro_measured_(nullptr),
        Dp_(nullptr), dp_(nullptr), Dp_out_(nullptr),
        Dv_(nullptr), dv_(nullptr), Dv_out_(nullptr),
        Dq_(nullptr), dq_(nullptr), Dq_out_(nullptr)
{
    // Set constant parts of Jacobians
    jacobian_delta_preint_.setIdentity(9,9);                                    // dDp'/dDp, dDv'/dDv, all zeros
    jacobian_delta_.setIdentity(9,9);                                           //
    jacobian_calib_.setZero(9,6);
}

ProcessorIMU::~ProcessorIMU()
{
//    std::cout << "destructed     -p-IMU" << id() << std::endl;
}

VectorXs ProcessorIMU::correctDelta(const Motion& _motion, const CaptureMotionPtr _capture)
{

    /* Correct measured delta: delta_corr = delta ++ J_bias * (bias - bias_preint)
     * where:
     *   delta       = pre-integrated delta at time dt
     *   J_bias      = Jacobian of the preintegrated delta at time dt
     *   bias        = current bias estimate
     *   bias_preint = bias estimate when we performed the pre-integration
     *   ++          = additive composition: x+dx for p and v, q*exp(dv) for the quaternion.
     */

    // Get current delta and Jacobian
    VectorXs delta_preint  = _motion.delta_integr_;
    MatrixXs J_bias        = _motion.jacobian_calib_;

    // Get current biases from the capture's origin frame
    FrameIMUPtr frame_origin   = std::static_pointer_cast<FrameIMU>(_capture->getOriginFramePtr());
    Vector6s bias; bias       << frame_origin->getAccBiasPtr()->getState(), frame_origin->getGyroBiasPtr()->getState();

    // Get preintegrated biases from the capture's feature
    FeatureIMUPtr feature              = std::static_pointer_cast<FeatureIMU>(_capture->getFeatureList().front());
    Vector6s bias_preint; bias_preint << feature->acc_bias_preint_, feature->gyro_bias_preint_;

    // Compute update step
    VectorXs delta_step = J_bias * (bias - bias_preint);

    // Correct delta
    VectorXs delta_correct(10);
    delta_correct.head(3)           = delta_preint.head(3) + delta_step.head(3);
    Map<const Quaternions> deltaq   (&delta_preint(3));
    Map<Quaternions> deltaq_correct (&delta_correct(3));
    deltaq_correct                  = deltaq * v2q(delta_step.segment(3,3));
    delta_correct.tail(3)           = delta_preint.tail(3) + delta_step.tail(3);

    return delta_correct;
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
    //WOLF_DEBUG( "Time span   : " , getBuffer().get().back().ts_ - getBuffer().get().front().ts_ );
    //WOLF_DEBUG( "BufferLength: " , getBuffer().get().size() );
    //WOLF_DEBUG( "AngleTurned : " , 2.0 * acos(delta_integrated_(6)) );
    if(!voting_active_)
        return false;
    // time span
    if (getBuffer().get().back().ts_ - getBuffer().get().front().ts_ > max_time_span_)
    {
//        WOLF_DEBUG( "PM: vote: time span" );
        return true;
    }
    // buffer length
    if (getBuffer().get().size() > max_buff_length_)
    {
//        WOLF_DEBUG( "PM: vote: buffer size" );
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

} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"

namespace wolf
{
WOLF_REGISTER_PROCESSOR("IMU", ProcessorIMU)
}
