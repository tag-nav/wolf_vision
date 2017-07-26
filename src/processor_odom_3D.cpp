#include "processor_odom_3D.h"
namespace wolf
{

ProcessorOdom3D::ProcessorOdom3D(ProcessorOdom3DParamsPtr _params, SensorOdom3DPtr _sensor_ptr) :
                ProcessorMotion("ODOM 3D", 7, 7, 6, 6),
                max_time_span_  (_params ? _params    ->max_time_span   : 1.0  ),
                max_buff_length_(_params ? _params    ->max_buff_length : 10   ),
                dist_traveled_  (_params ? _params    ->dist_traveled   : 1.0  ),
                angle_turned_   (_params ? _params    ->angle_turned    : 0.5  ),
                p1_(nullptr), p2_(nullptr), p_out_(nullptr),
                q1_(nullptr), q2_(nullptr), q_out_(nullptr)
        {
            setup(_sensor_ptr);
            jacobian_delta_preint_.setZero(delta_cov_size_, delta_cov_size_);
            jacobian_delta_.setZero(delta_cov_size_, delta_cov_size_);
        }


ProcessorOdom3D::~ProcessorOdom3D()
{
}

void ProcessorOdom3D::setup(SensorOdom3DPtr sen_ptr)
{
    if (sen_ptr)
    {
        // we steal the parameters from the provided odom3D sensor.
        k_disp_to_disp_ = sen_ptr->getDispVarToDispNoiseFactor();
        k_disp_to_rot_ = sen_ptr->getDispVarToRotNoiseFactor();
        k_rot_to_rot_ = sen_ptr->getRotVarToRotNoiseFactor();
        min_disp_var_ = sen_ptr->getMinDispVar();
        min_rot_var_ = sen_ptr->getMinRotVar();
    }
    else
    {
        // we put default params.
        k_disp_to_disp_ =   0;
        k_disp_to_rot_  =   0;
        k_rot_to_rot_   =   0;
        min_disp_var_   = 0.1; // around 30cm error
        min_rot_var_    = 0.1; // around 9 degrees error
    }
}

void ProcessorOdom3D::data2delta(const Eigen::VectorXs& _data,
                                 const Eigen::MatrixXs& _data_cov,
                                 const Scalar _dt,
                                 Eigen::VectorXs& _delta,
                                 Eigen::MatrixXs& _delta_cov,
                                 const Eigen::VectorXs& _calib,
                                 Eigen::MatrixXs& _jacobian_calib)
{
    assert((_data.size() == 6 || _data.size() == 7) && "Wrong data size. Must be 6 or 7 for 3D.");
    Scalar disp, rot; // displacement and rotation of this motion step
    if (_data.size() == (long int)6)
    {
        // rotation in vector form
        _delta.head<3>() = _data.head<3>();
        new (&q_out_) Eigen::Map<Eigen::Quaternions>(_delta.data() + 3);
        q_out_ = v2q(_data.tail<3>());
        disp = _data.head<3>().norm();
        rot = _data.tail<3>().norm();
    }
    else
    {
        // rotation in quaternion form
        _delta = _data;
        disp = _data.head<3>().norm();
        rot = 2 * acos(_data(3));
    }
    /* Jacobians of d = data2delta(data, dt)
     * with: d =    [Dp Dq]
     *       data = [dp do]
     *
     *       Dp = dp
     *       Dq = v2q(do)
     *
     * dDp/ddp = I
     * dDp/ddo = 0
     * dDo/ddp = 0
     * dDo/ddo = I
     *
     * so, J = I, and delta_cov = _data_cov
     */
    // We discard _data_cov and create a new one from the measured motion
    Scalar disp_var = min_disp_var_ + k_disp_to_disp_ * disp;
    Scalar rot_var = min_rot_var_ + k_disp_to_rot_ * disp + k_rot_to_rot_ * rot;
    Eigen::Matrix6s data_cov(Eigen::Matrix6s::Identity());
    data_cov(0, 0) = data_cov(1, 1) = data_cov(2, 2) = disp_var;
    data_cov(3, 3) = data_cov(4, 4) = data_cov(5, 5) = rot_var;
    _delta_cov = data_cov;
}

void ProcessorOdom3D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, const Scalar _Dt2,
                                     Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                     Eigen::MatrixXs& _jacobian2)
{
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");
    assert(_jacobian1.rows() == delta_cov_size_ && _jacobian1.cols() == delta_cov_size_ && "Wrong _jacobian1 size");
    assert(_jacobian2.rows() == delta_cov_size_ && _jacobian2.cols() == delta_cov_size_ && "Wrong _jacobian2 size");
    remap(_delta1, _delta2, _delta1_plus_delta2);
    /* Jacobians of D' = D (+) d
     * with: D = [Dp Dq]
     *       d = [dp dq]
     *
     * dDp'/dDp = I
     * dDp'/dDo = -DR * skew(dp)   // (Sola 16, ex. B.3.2 and Sec. 7.2.3)
     * dDo'/dDp = 0
     * dDo'/dDo = dR.tr            // (Sola 16, Sec. 7.2.3)
     *
     * dDp'/ddp = DR
     * dDp'/ddo = 0
     * dDo'/ddp = 0
     * dDo'/ddo = I
     */

    // temporaries
    Eigen::Matrix3s DR = q1_.matrix();
    Eigen::Matrix3s dR = q2_.matrix();

    // fill Jacobians
    _jacobian1.setIdentity();
    _jacobian1.block<3, 3>(0, 3) = -DR * skew(p2_); // (Sola 16, ex. B.3.2 and Sec. 7.2.3)
    _jacobian1.block<3, 3>(3, 3) = dR.transpose(); // (Sola 16, Sec. 7.2.3)

    _jacobian2.setIdentity();
    _jacobian2.block<3, 3>(0, 0) = DR; // (Sola 16, Sec. 7.2.3)

    // perform composition here to avoid aliasing problems if _delta1 and _delta_plus_delta share the same storage
    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;
}

void ProcessorOdom3D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, const Scalar _Dt2,
                                     Eigen::VectorXs& _delta1_plus_delta2)
{
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");
    remap(_delta1, _delta2, _delta1_plus_delta2);
    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;
}

void ProcessorOdom3D::statePlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _Dt,
                                 Eigen::VectorXs& _x_plus_delta)
{   
    assert(_x.size() >= x_size_ && "Wrong _x vector size"); //we need a state vector which size is at least x_size_
    assert(_delta.size() == delta_size_ && "Wrong _delta vector size");
    assert(_x_plus_delta.size() == x_size_ && "Wrong _x_plus_delta vector size");
    remap(_x.head(x_size_), _delta, _x_plus_delta); //we take only the x_sixe_ first elements of the state Vectors (Position + Orientation)
    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;
}

Motion ProcessorOdom3D::interpolate(const Motion& _motion_ref, Motion& _motion_second, TimeStamp& _ts)
{

//    WOLF_TRACE("motion ref ts: ", _motion_ref.ts_.get());
//    WOLF_TRACE("interp ts    : ", _ts.get());
//    WOLF_TRACE("motion ts    : ", _motion_second.ts_.get());
//
//    WOLF_TRACE("ref delta size: ", _motion_ref.delta_.size(), " , cov size: ", _motion_ref.delta_cov_.size());
//    WOLF_TRACE("ref Delta size: ", _motion_ref.delta_integr_.size(), " , cov size: ", _motion_ref.delta_integr_cov_.size());
//    WOLF_TRACE("sec delta size: ", _motion_second.delta_.size(), " , cov size: ", _motion_second.delta_cov_.size());
//    WOLF_TRACE("sec Delta size: ", _motion_second.delta_integr_.size(), " , cov size: ", _motion_second.delta_integr_cov_.size());

    // resolve out-of-bounds time stamp as if the time stamp was exactly on the bounds
    if (_ts <= _motion_ref.ts_)     // behave as if _ts == _motion_ref.ts_
    { // return null motion. Second stays the same.
        Motion motion_int(_motion_ref);
        motion_int.delta_ = deltaZero();
        motion_int.delta_cov_.setZero();
        return motion_int;
    }
    if (_motion_second.ts_ <= _ts)  // behave as if _ts == _motion_second.ts_
    { // return original second motion. Second motion becomes null motion
        Motion motion_int(_motion_second);
        _motion_second.delta_ = deltaZero();
        _motion_second.delta_cov_.setZero();
        return motion_int;
    }

    assert(_motion_ref.ts_ <= _ts && "Interpolation time stamp out of bounds");
    assert(_ts <= _motion_second.ts_ && "Interpolation time stamp out of bounds");

    assert(_motion_ref.delta_.size() == delta_size_ && "Wrong delta size");
    assert(_motion_ref.delta_cov_.cols() == delta_cov_size_ && "Wrong delta cov size");
    assert(_motion_ref.delta_cov_.rows() == delta_cov_size_ && "Wrong delta cov size");
    assert(_motion_ref.delta_integr_.size() == delta_size_ && "Wrong delta size");
//    assert(_motion_ref.delta_integr_cov_.cols() == delta_cov_size_ && "Wrong delta cov size");
//    assert(_motion_ref.delta_integr_cov_.rows() == delta_cov_size_ && "Wrong delta cov size");
    assert(_motion_second.delta_.size() == delta_size_ && "Wrong delta size");
    assert(_motion_second.delta_cov_.cols() == delta_cov_size_ && "Wrong delta cov size");
    assert(_motion_second.delta_cov_.rows() == delta_cov_size_ && "Wrong delta cov size");
    assert(_motion_second.delta_integr_.size() == delta_size_ && "Wrong delta size");
//    assert(_motion_second.delta_integr_cov_.cols() == delta_cov_size_ && "Wrong delta cov size");
//    assert(_motion_second.delta_integr_cov_.rows() == delta_cov_size_ && "Wrong delta cov size");


    using namespace Eigen;
    // Interpolate between motion_ref and motion, as in:
    //
    // motion_ref ------ ts_ ------ motion
    //                 return
    //
    // and return the value at the given time_stamp ts_.
    //
    // The position receives linear interpolation:
    //    p_ret = (ts - t_ref) / dt * (p - p_ref)
    //
    // the quaternion receives a slerp interpolation
    //    q_ret = q_ref.slerp( (ts - t_ref) / dt , q);
    //
    // See extensive documentation in ProcessorMotion::interpolate().

    // reference
    TimeStamp           t_ref       = _motion_ref.ts_;

    // final
    TimeStamp           t_sec       = _motion_second.ts_;
    Map<VectorXs>       dp_sec(_motion_second.delta_.data(), 3);
    Map<Quaternions>    dq_sec(_motion_second.delta_.data() + 3);

    // interpolated
    Motion              motion_int  = motionZero(_ts);
    Map<VectorXs>       dp_int(motion_int.delta_.data(), 3);
    Map<Quaternions>    dq_int(motion_int.delta_.data() + 3);

    // Jacobians for covariance propagation
    MatrixXs            J_ref(delta_cov_size_, delta_cov_size_);
    MatrixXs            J_int(delta_cov_size_, delta_cov_size_);

    // interpolate delta
    Scalar     tau  = (_ts - t_ref) / (t_sec - t_ref); // interpolation factor (0 to 1)
    motion_int.ts_  = _ts;
    dp_int          = tau * dp_sec;
    dq_int          = Quaternions::Identity().slerp(tau, dq_sec);
    deltaPlusDelta(_motion_ref.delta_integr_, motion_int.delta_, (t_sec - t_ref), motion_int.delta_integr_, J_ref, J_int);

    // interpolate covariances
    motion_int.delta_cov_           = tau * _motion_second.delta_cov_;
//    motion_int.delta_integr_cov_    = J_ref * _motion_ref.delta_integr_cov_ * J_ref.transpose() + J_int * _motion_second.delta_cov_ * J_int.transpose();

    // update second delta ( in place update )
    dp_sec          = dq_int.conjugate() * ((1 - tau) * dp_sec);
    dq_sec          = dq_int.conjugate() * dq_sec;
    _motion_second.delta_cov_ = (1 - tau) * _motion_second.delta_cov_; // easy interpolation // TODO check for correctness
    //Dp            = Dp; // trivial, just leave the code commented
    //Dq            = Dq; // trivial, just leave the code commented
    //_motion.delta_integr_cov_ = _motion.delta_integr_cov_; // trivial, just leave the code commented

    return motion_int;
}

ProcessorBasePtr ProcessorOdom3D::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sen_ptr)
{
    // cast inputs to the correct type
    std::shared_ptr<ProcessorOdom3DParams> prc_odo_params = std::static_pointer_cast<ProcessorOdom3DParams>(_params);

    SensorOdom3DPtr sen_odo =std::static_pointer_cast<SensorOdom3D>(_sen_ptr);

    // construct processor
    ProcessorOdom3DPtr prc_odo = std::make_shared<ProcessorOdom3D>(prc_odo_params, sen_odo);

    // setup processor
    prc_odo->setName(_unique_name);

    return prc_odo;
}

bool ProcessorOdom3D::voteForKeyFrame()
{
    //WOLF_DEBUG( "Time span   : " , getBuffer().get().back().ts_ - getBuffer().get().front().ts_ );
    //WOLF_DEBUG( " last ts : ", getBuffer().get().back().ts_);
    //WOLF_DEBUG( " first ts : ", getBuffer().get().front().ts_);
    //WOLF_DEBUG( "BufferLength: " , getBuffer().get().size() );
    //WOLF_DEBUG( "DistTraveled: " , delta_integrated_.head(3).norm() );
    //WOLF_DEBUG( "AngleTurned : " , 2.0 * acos(delta_integrated_(6)) );
    // time span
    if (getBuffer().get().back().ts_ - getBuffer().get().front().ts_ > max_time_span_)
    {
        WOLF_DEBUG( "PM: vote: time span" );
        return true;
    }
    // buffer length
    if (getBuffer().get().size() > max_buff_length_)
    {
        WOLF_DEBUG( "PM: vote: buffer size" );
        return true;
    }
    // distance traveled
    Scalar dist = delta_integrated_.head(3).norm();
    if (dist > dist_traveled_)
    {
        WOLF_DEBUG( "PM: vote: distance traveled" );
        return true;
    }
    // angle turned
    Scalar angle = 2.0 * acos(delta_integrated_(6));
    if (angle > angle_turned_)
    {
        WOLF_DEBUG( "PM: vote: angle turned" );
        return true;
    }
    //WOLF_DEBUG( "PM: do not vote" );
    return false;
}

}


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("ODOM 3D", ProcessorOdom3D)
} // namespace wolf
