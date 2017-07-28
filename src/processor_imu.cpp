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
