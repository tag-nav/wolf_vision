#include "processor_imu.h"

namespace wolf {

ProcessorIMU::ProcessorIMU(ProcessorIMUParamsPtr _params) :
        ProcessorMotion("IMU", 16, 10, 9, 6),
        max_time_span_  (_params ? _params    ->max_time_span   : 1.0  ),
        max_buff_length_(_params ? _params    ->max_buff_length : 10000   ),
        dist_traveled_  (_params ? _params    ->dist_traveled   : 1.0  ),
        angle_turned_   (_params ? _params    ->angle_turned    : 0.2  ),
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

void ProcessorIMU::process(CaptureBasePtr _incoming_ptr)
{
    incoming_ptr_ = std::static_pointer_cast<CaptureMotion>(_incoming_ptr);

    preProcess();
    integrate();

    if (voteForKeyFrame() && permittedKeyFrame())
    {
        // key_capture
        //        CaptureMotionPtr key_capture_ptr = last_ptr_;
        //        FrameBasePtr key_frame_ptr = key_capture_ptr->getFramePtr();
        FrameIMUPtr key_frame_ptr = std::static_pointer_cast<FrameIMU>(last_ptr_->getFramePtr());

        // Set the frame of key_capture as key
        key_frame_ptr->setState(getCurrentState());
        key_frame_ptr->setTimeStamp(getCurrentTimeStamp());
        key_frame_ptr->setKey();

        // create motion feature and add it to the key_capture
        FeatureIMUPtr key_feature_ptr = std::make_shared<FeatureIMU>(
                last_ptr_->getBuffer().get().back().delta_integr_,
                last_ptr_->getBuffer().get().back().delta_integr_cov_.determinant() > 0 ?
                        last_ptr_->getBuffer().get().back().delta_integr_cov_ :
                        Eigen::MatrixXs::Identity(delta_cov_size_, delta_cov_size_) * 1e-4, // avoid a strict zero in the covariance
                key_frame_ptr->getAccBiasPtr()->getVector(),
                key_frame_ptr->getGyroBiasPtr()->getVector(),
                this->getJacobians()); 
        last_ptr_->addFeature(std::static_pointer_cast<FeatureBase>(key_feature_ptr));

        // create motion constraint and link it to parent feature and other frame (which is origin's frame)
        auto ctr_ptr    =  emplaceConstraint(key_feature_ptr, origin_ptr_->getFramePtr());
//        key_feature_ptr -> addConstraint(ctr_ptr);
//        origin_ptr_->getFramePtr() -> addConstrainedBy(ctr_ptr);

        // new last capture
        CaptureMotionPtr new_capture_ptr = std::make_shared<CaptureMotion>(key_frame_ptr->getTimeStamp(),
                                                    getSensorPtr(),
                                                    Eigen::VectorXs::Zero(data_size_),
                                                    Eigen::MatrixXs::Zero(data_size_, data_size_),
                                                    key_frame_ptr);
        // reset the new buffer
        new_capture_ptr->getBuffer().get().push_back(Motion( {key_frame_ptr->getTimeStamp(), deltaZero(), deltaZero(),
                                             Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_),
                                             Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_)}));


        // create a new frame
        FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME, key_frame_ptr->getState(), new_capture_ptr->getTimeStamp());
        new_frame_ptr->addCapture(new_capture_ptr); // Add Capture to the new Frame

        // reset integrals
        delta_integrated_ = deltaZero();
        delta_integrated_cov_.setZero();

        // reset processor origin to the new keyframe's capture
        origin_ptr_ = last_ptr_;
        last_ptr_ = new_capture_ptr;

        // reset derived things
        resetDerived();

        // callback to other processors
        getProblem()->keyFrameCallback(key_frame_ptr, shared_from_this(), time_tolerance_);

    }


    postProcess();

    // clear incoming just in case
    incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
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
    WOLF_DEBUG( "Time span   : " , getBuffer().get().back().ts_ - getBuffer().get().front().ts_ );
    WOLF_DEBUG( "BufferLength: " , getBuffer().get().size() );
    WOLF_DEBUG( "DistTraveled: " , delta_integrated_.head(3).norm() );
    WOLF_DEBUG( "AngleTurned : " , 2.0 * acos(delta_integrated_(6)) );
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
    WOLF_DEBUG( "PM: do not vote" );
    return false;
}

} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"

namespace wolf
{
WOLF_REGISTER_PROCESSOR("IMU", ProcessorIMU)
}
