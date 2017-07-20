#include "processor_imu.h"

namespace wolf {

ProcessorIMU::ProcessorIMU(ProcessorIMUParamsPtr _params) :
        ProcessorMotion("IMU", 16, 10, 9, 6),
        max_time_span_  (_params ? _params    ->max_time_span   : 1.0  ),
        max_buff_length_(_params ? _params    ->max_buff_length : 10000   ),
        dist_traveled_  (_params ? _params    ->dist_traveled   : 1.0  ),
        angle_turned_   (_params ? _params    ->angle_turned    : 0.2  ),
        voting_active_  (_params ? _params    ->voting_active    : false  ),
        frame_imu_ptr_(nullptr),
        gravity_(wolf::gravity()),
        acc_bias_(Eigen::Vector3s::Zero()),
        gyro_bias_(Eigen::Vector3s::Zero()),
        acc_measured_(nullptr),
        gyro_measured_(nullptr),
        Dp_(nullptr), dp_(nullptr), Dp_out_(nullptr),
        Dv_(nullptr), dv_(nullptr), Dv_out_(nullptr),
        Dq_(nullptr), dq_(nullptr), Dq_out_(nullptr)
{
    // Set constant parts of Jacobians
    jacobian_delta_preint_.setIdentity(9,9);                                    // dDp'/dDp, dDv'/dDv, all zeros
    jacobian_delta_.setIdentity(9,9);                                           //
}

ProcessorIMU::~ProcessorIMU()
{
//    std::cout << "destructed     -p-IMU" << id() << std::endl;
}

VectorXs ProcessorIMU::correctDelta(const Motion& _motion, Scalar _dt, const CaptureMotionPtr _capture)
{

    // Full delta time interval
    Scalar Dt = _capture->getTimeStamp() - _capture->getOriginFramePtr()->getTimeStamp();
    // Linear interpolation factor
    Scalar alpha = Dt > Constants::EPS_SMALL ? _dt/Dt : 1.0;

    /* Correct measured delta: delta_corr = delta + alpha * J_bias * (bias - bias_preint)
     * where:
     *   delta       = pre-integrated delta at time dt
     *   J_bias      = Jacobian of the preintegrated delta at time Dt
     *   alpha       = interpolation factor from dt=0 to dt=Dt
     *   bias        = current bias estimate
     *   bias_preint = bias estimate when we performed the pre-integration
     */

    // Get current biases
    FrameIMUPtr frame = std::static_pointer_cast<FrameIMU>(_capture->getFramePtr());
    Vector3s ab = frame->getAccBiasPtr()->getState();
    Vector3s wb = frame->getGyroBiasPtr()->getState();

    // Get the only feature in the capture
    FeatureIMUPtr feature = std::static_pointer_cast<FeatureIMU>(_capture->getFeatureList().front());

    // Compute bias change
    Vector3s dab = ab - feature->acc_bias_preint_;
    Vector3s dwb = wb - feature->gyro_bias_preint_;
//    Vector6s db; db << dab, dwb;
//
//    // Get the Jacobian // TODO get it from the Motion
//    MatrixXs J(9,6);
//    J.topLeftCorner(3,3) = feature->dDp_dab_;
//    J.topRightCorner(3,3) = feature->dDp_dwb_;
//    J.block(3,0,3,3) = Matrix3s::Zero();
//    J.block(3,3,3,3) = feature->dDq_dwb_;
//    J.bottomLeftCorner(3,3) = feature->dDv_dab_;
//    J.bottomRightCorner(3,3) = feature->dDv_dwb_;
//    J *= alpha; // linear intermpolation
//
//    // Do the correction
//    VectorXs delta_corr_tangent(9);
//    delta_corr_tangent = delta + J * db;


    VectorXs delta = _motion.delta_integr_;
    VectorXs delta_correct(10);
    // P
    delta_correct.head(3)      = delta.head(3) + alpha * feature->dDp_dab_ * dab + alpha * feature->dDp_dwb_ * dwb;
    // Q
    Eigen::Vector3s do_step    = alpha * feature->dDq_dwb_ * dwb;
    Map<const Quaternions> dq(delta.data() + 3);
    Map<Quaternions> dq_correct(delta_correct.data() + 3);
    dq_correct = dq * v2q(do_step);
    // V
    delta_correct.tail(3)      = delta.tail(3) + alpha * feature->dDv_dab_ * dab + alpha * feature->dDv_dwb_ * dwb;

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
        WOLF_DEBUG( "PM: vote: time span" );
        return true;
    }
    // buffer length
    if (getBuffer().get().size() > max_buff_length_)
    {
        WOLF_DEBUG( "PM: vote: buffer size" );
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

} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"

namespace wolf
{
WOLF_REGISTER_PROCESSOR("IMU", ProcessorIMU)
}
