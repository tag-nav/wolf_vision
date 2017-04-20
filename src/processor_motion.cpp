#include "processor_motion.h"
namespace wolf
{

ProcessorMotion::ProcessorMotion(const std::string& _type, Size _state_size, Size _delta_size,
                                 Size _delta_cov_size, Size _data_size, const Scalar& _time_tolerance) :
        ProcessorBase(_type, _time_tolerance),
        x_size_(_state_size),
        delta_size_(_delta_size),
        delta_cov_size_(_delta_cov_size),
        data_size_(_data_size),
        origin_ptr_(),
        last_ptr_(),
        incoming_ptr_(),
        dt_(0.0), x_(_state_size),
        delta_(_delta_size),
        delta_cov_(_delta_cov_size, _delta_cov_size),
        delta_integrated_(_delta_size),
        delta_integrated_cov_(_delta_cov_size, _delta_cov_size),
        data_(_data_size),
        jacobian_delta_preint_(delta_cov_size_, delta_cov_size_),
        jacobian_delta_(delta_cov_size_, delta_cov_size_)
{
    status_ = IDLE;
    //
}

ProcessorMotion::~ProcessorMotion()
{
//    std::cout << "destructed     -p-Mot" << id() << std::endl;
}

void ProcessorMotion::process(CaptureBasePtr _incoming_ptr)
{


    if (status_ == IDLE)
    {
//        std::cout << "PM: IDLE" << std::endl;
        TimeStamp t0 = _incoming_ptr->getTimeStamp();

        if (origin_ptr_ == nullptr)
        {
            FrameBasePtr frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t0);
            if (frm && fabs(frm->getTimeStamp() - t0) < time_tolerance_)
            {
                std::cout << "PM: join KF" << std::endl;
                // Join existing KF
                setOrigin(frm);
            }
            else
            {
                // Create new KF for origin
                std::cout << "PM: make KF" << std::endl;
                VectorXs x0 = getProblem()->zeroState();
                setOrigin(x0, t0);
            }
        }
        status_ = RUNNING;
//        std::cout << "PM: RUNNING" << std::endl;
    }


    incoming_ptr_ = std::static_pointer_cast<CaptureMotion>(_incoming_ptr);

    preProcess();

    // integrate data
    integrateOneStep();

    // Update state and time stamps
    last_ptr_->setTimeStamp(incoming_ptr_->getTimeStamp());
    last_ptr_->getFramePtr()->setTimeStamp(last_ptr_->getTimeStamp());
    last_ptr_->getFramePtr()->setState(getCurrentState());

    if (voteForKeyFrame() && permittedKeyFrame())
    {
        // Set the frame of last_ptr as key
        FrameBasePtr key_frame_ptr = last_ptr_->getFramePtr();
        key_frame_ptr->setState(getCurrentState());
        key_frame_ptr->setTimeStamp(getCurrentTimeStamp());
        key_frame_ptr->setKey();

        // create motion feature and add it to the key_capture
        delta_integrated_cov_ = integrateBufferCovariance(last_ptr_->getBuffer());
        FeatureBasePtr key_feature_ptr = std::make_shared<FeatureBase>("MOTION", last_ptr_->getBuffer().get().back().delta_integr_, delta_integrated_cov_);
        last_ptr_->addFeature(key_feature_ptr);

        // create motion constraint and link it to parent feature and other frame (which is origin's frame)
        auto ctr_ptr = emplaceConstraint(key_feature_ptr, origin_ptr_->getFramePtr());

        // new capture
        CaptureMotionPtr new_capture_ptr = std::make_shared<CaptureMotion>(key_frame_ptr->getTimeStamp(),
                                                    getSensorPtr(),
                                                    Eigen::VectorXs::Zero(data_size_),
                                                    Eigen::MatrixXs::Zero(data_size_, data_size_),
                                                    delta_size_, delta_cov_size_,
                                                    key_frame_ptr);
        // reset the new buffer
        new_capture_ptr->getBuffer().get().push_back( motionZero(key_frame_ptr->getTimeStamp()) ) ;

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

CaptureMotionPtr ProcessorMotion::findCaptureContainingTimeStamp(const TimeStamp& _ts) const
{
    //std::cout << "ProcessorMotion::findCaptureContainingTimeStamp: ts = " << _ts.getSeconds() << "." << _ts.getNanoSeconds() << std::endl;
    auto capture_ptr = last_ptr_;
    while (capture_ptr != nullptr)
    {
        // capture containing motion previous than the ts found:
        if (capture_ptr->getBuffer().get().front().ts_ < _ts)
            return capture_ptr;
        else
        {
            // go to the previous motion capture
            if (capture_ptr == last_ptr_)
                capture_ptr = std::static_pointer_cast<CaptureMotion>(origin_ptr_);
            else if (capture_ptr->getOriginFramePtr() == nullptr)
                return nullptr;
            else
            {
                CaptureBasePtr capture_base_ptr = capture_ptr->getOriginFramePtr()->getCaptureOf(getSensorPtr());
                if (capture_base_ptr == nullptr)
                    return nullptr;
                else
                    capture_ptr = std::static_pointer_cast<CaptureMotion>(capture_base_ptr);
            }
        }
    }
    return capture_ptr;
}

void ProcessorMotion::setOrigin(const Eigen::VectorXs& _x_origin, const TimeStamp& _ts_origin)
{
    FrameBasePtr key_frame_ptr = getProblem()->emplaceFrame(KEY_FRAME, _x_origin, _ts_origin);
    setOrigin(key_frame_ptr);
}

void ProcessorMotion::setOrigin(FrameBasePtr _origin_frame)
{

    assert(_origin_frame->getTrajectoryPtr() != nullptr
            && "ProcessorMotion::setOrigin: origin frame must be in the trajectory.");
    assert(_origin_frame->isKey() && "ProcessorMotion::setOrigin: origin frame must be KEY FRAME.");

    // make (empty) origin Capture
    origin_ptr_ = std::make_shared<CaptureMotion>(_origin_frame->getTimeStamp(),
                                                  getSensorPtr(),
                                                  Eigen::VectorXs::Zero(data_size_),
                                                  Eigen::MatrixXs::Zero(data_size_, data_size_),
                                                  delta_size_, delta_cov_size_,
                                                  nullptr);
    // Add origin capture to origin frame
    _origin_frame->addCapture(origin_ptr_);

    // make (emtpy) last Capture
    last_ptr_ = std::make_shared<CaptureMotion>(_origin_frame->getTimeStamp(),
                                                getSensorPtr(),
                                                Eigen::VectorXs::Zero(data_size_),
                                                Eigen::MatrixXs::Zero(data_size_, data_size_),
                                                delta_size_, delta_cov_size_,
                                                _origin_frame);
    // Make non-key-frame at last Capture
    FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME,
                                                           _origin_frame->getState(),
                                                           _origin_frame->getTimeStamp());
    new_frame_ptr->addCapture(last_ptr_);

    /* Status:
     * KF --- F ---
     * o      l     i
     */

    // Reset deltas
    delta_ = deltaZero();
    delta_integrated_ = deltaZero();
    delta_cov_.setZero();
    delta_integrated_cov_.setZero();

    // clear and reset buffer
    getBuffer().get().clear();
    getBuffer().get().push_back(motionZero(_origin_frame->getTimeStamp()));

    // Reset derived things
    resetDerived();
}

bool ProcessorMotion::keyFrameCallback(FrameBasePtr _new_keyframe, const Scalar& _time_tol_other)
{
//    std::cout << "PM: KF" << _new_keyframe->id() << " callback received at ts= " << _new_keyframe->getTimeStamp().get() << std::endl;
//    std::cout << "    while last ts= " << last_ptr_->getTimeStamp().get() << std::endl;

    assert(_new_keyframe->getTrajectoryPtr() != nullptr
            && "ProcessorMotion::keyFrameCallback: key frame must be in the trajectory.");

    // get keyframe's time stamp
    TimeStamp new_ts = _new_keyframe->getTimeStamp();

    // find capture whose buffer is affected by the new keyframe
    CaptureMotionPtr existing_capture = findCaptureContainingTimeStamp(new_ts);
    assert(existing_capture != nullptr
            && "ProcessorMotion::keyFrameCallback: no motion capture containing the required TimeStamp found");

    // Find the frame acting as the capture's origin
    FrameBasePtr new_keyframe_origin = existing_capture->getOriginFramePtr();

    // create motion capture
    CaptureMotionPtr new_capture = std::make_shared<CaptureMotion>(new_ts, getSensorPtr(),
                                                                         Eigen::VectorXs::Zero(data_size_),
                                                                         Eigen::MatrixXs::Zero(data_size_, data_size_),
                                                                         delta_size_, delta_cov_size_,
                                                                         new_keyframe_origin);
    // add motion capture to keyframe
    _new_keyframe->addCapture(new_capture);

    // split the buffer
    // and give the old buffer to the key_capture
    existing_capture->getBuffer().split(new_ts, new_capture->getBuffer());

    // interpolate individual delta
    if (!existing_capture->getBuffer().get().empty() && new_capture->getBuffer().get().back().ts_ != new_ts)
    {
        // interpolate Motion at the new time stamp
        Motion motion_interpolated = interpolate(new_capture->getBuffer().get().back(), // last Motion of old buffer
                                 existing_capture->getBuffer().get().front(), // first motion of new buffer
                                 new_ts);
        // add to old buffer
        new_capture->getBuffer().get().push_back(motion_interpolated);
    }

    Eigen::MatrixXs new_covariance = integrateBufferCovariance(new_capture->getBuffer());

    // check for very small covariances and fix
    // FIXME: This situation means no motion. Therefore, two KFs have been created at the same TS: the motion KF, and the other KF. Making a factor with nearly no cov is OK, but an overkill for a situation that should not have appeared.
    if (new_covariance.determinant() < Constants::EPS_SMALL)
    {
        WOLF_DEBUG("Bad motion covariance determinant: ", new_covariance.determinant());
        new_covariance += MatrixXs::Identity(delta_cov_size_, delta_cov_size_)*1e-4;
        WOLF_DEBUG("Fixed motion covariance determinant: ", new_covariance.determinant());
    }

    // create motion feature and add it to the capture
    FeatureBasePtr new_feature = std::make_shared<FeatureBase>(
            "ODOM 2D",
            new_capture->getBuffer().get().back().delta_integr_,
            new_covariance);

    new_capture->addFeature(new_feature);

    // create motion constraint and add it to the feature, and link it to the other frame (origin)
    emplaceConstraint(new_feature, new_keyframe_origin);



    /////////////////////////////////////////////////////////
    // Update the existing capture
    if (existing_capture == last_ptr_)
        // reset processor origin
        origin_ptr_ = new_capture;

    existing_capture->setOriginFramePtr(_new_keyframe);

    // reintegrate existing buffer -- note: the result of re-integration is stored in the same buffer!
    reintegrateBuffer(existing_capture);

    // modify existing feature and constraint (if they exist in the existing capture)
    if (!existing_capture->getFeatureList().empty())
    {
        FeatureBasePtr existing_feature = existing_capture->getFeatureList().back(); // there is only one feature!

        // Modify existing feature --------
        existing_feature->setMeasurement(existing_capture->getBuffer().get().back().delta_integr_);
        MatrixXs existing_covariance = integrateBufferCovariance(existing_capture->getBuffer());
        existing_feature->setMeasurementCovariance(existing_covariance);

        // Modify existing constraint --------
        // Instead of modifying, we remove one ctr, and create a new one.
        auto ctr_to_remove = existing_feature->getConstraintList().back(); // there is only one constraint!
        auto new_ctr = emplaceConstraint(existing_feature, _new_keyframe);
        ctr_to_remove->remove();  // remove old constraint now (otherwise c->remove() gets propagated to f, C, F, etc.)
    }

    return true;
}

void ProcessorMotion::integrateOneStep()
{
    // Set dt
    updateDt();

    // get data and convert it to delta, and obtain also the delta covariance
    data2delta(incoming_ptr_->getData(), incoming_ptr_->getDataCovariance(), dt_);

    // integrate the current delta to pre-integrated measurements, and get Jacobians
    deltaPlusDelta(getBuffer().get().back().delta_integr_, delta_, dt_, delta_integrated_, jacobian_delta_preint_, jacobian_delta_);

    // push all into buffer
    getBuffer().get().push_back(Motion( {incoming_ptr_->getTimeStamp(), delta_, delta_integrated_,
                                         jacobian_delta_, jacobian_delta_preint_, delta_cov_}));

}

void ProcessorMotion::reintegrateBuffer(CaptureMotionPtr _capture_ptr)
{

    // start with empty motion
    _capture_ptr->getBuffer().get().push_front(motionZero(_capture_ptr->getOriginFramePtr()->getTimeStamp()));
    auto motion_it = _capture_ptr->getBuffer().get().begin();
    auto prev_motion_it = motion_it;
    motion_it++;
    while (motion_it != _capture_ptr->getBuffer().get().end())
    {
        // get dt
        const Scalar dt = motion_it->ts_ - prev_motion_it->ts_;

        // integrate delta into delta_integr, and rewrite the buffer
        deltaPlusDelta(prev_motion_it->delta_integr_, motion_it->delta_, dt, motion_it->delta_integr_,
                       motion_it->jacobian_delta_integr_, motion_it->jacobian_delta_);

        // advance in buffer
        motion_it++;
        prev_motion_it++;
    }
}

Eigen::MatrixXs ProcessorMotion::integrateBufferCovariance(const MotionBuffer& _motion_buffer)
{
    return _motion_buffer.integrateCovariance();
}


}
