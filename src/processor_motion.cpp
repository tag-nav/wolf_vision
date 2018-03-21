#include "processor_motion.h"
namespace wolf
{

ProcessorMotion::ProcessorMotion(const std::string& _type,
                                 Scalar _time_tolerance,
                                 Size _state_size,
                                 Size _delta_size,
                                 Size _delta_cov_size,
                                 Size _data_size,
                                 Size _calib_size) :
        ProcessorBase(_type, _time_tolerance),
        processing_step_(RUNNING_WITHOUT_PACK),
        x_size_(_state_size),
        data_size_(_data_size),
        delta_size_(_delta_size),
        delta_cov_size_(_delta_cov_size),
        calib_size_(_calib_size),
        origin_ptr_(),
        last_ptr_(),
        incoming_ptr_(),
        dt_(0.0), x_(_state_size),
        data_(_data_size),
        delta_(_delta_size),
        delta_cov_(_delta_cov_size, _delta_cov_size),
        delta_integrated_(_delta_size),
        delta_integrated_cov_(_delta_cov_size, _delta_cov_size),
        calib_(_calib_size),
        jacobian_delta_preint_(delta_cov_size_, delta_cov_size_),
        jacobian_delta_(delta_cov_size_, delta_cov_size_),
        jacobian_calib_(delta_size_, calib_size_)
{
    //
}

ProcessorMotion::~ProcessorMotion()
{
//    std::cout << "destructed     -p-Mot" << id() << std::endl;
}

void ProcessorMotion::process(CaptureBasePtr _incoming_ptr)
{
    if (_incoming_ptr == nullptr)
    {
        WOLF_ERROR("Received capture is nullptr.");
        return;
    }

    incoming_ptr_ = std::static_pointer_cast<CaptureMotion>(_incoming_ptr);

    preProcess(); // Derived class operations

    KFPackPtr pack = computeProcessingStep();
    if (pack)
        kf_pack_buffer_.removeUpTo( pack->key_frame->getTimeStamp() );

    switch(processing_step_)
    {

        case RUNNING_WITHOUT_PACK :
        case RUNNING_WITH_PACK_ON_ORIGIN :
            break;

        case RUNNING_WITH_PACK_BEFORE_ORIGIN :
        {
            // extract pack elements
            FrameBasePtr keyframe_from_callback = pack->key_frame;
            TimeStamp ts_from_callback = keyframe_from_callback->getTimeStamp();

            // find the capture whose buffer is affected by the new keyframe
            auto existing_capture = findCaptureContainingTimeStamp(ts_from_callback);

            // Find the frame acting as the capture's origin
            auto keyframe_origin = existing_capture->getOriginFramePtr();

            // emplace a new motion capture to the new keyframe
            auto capture_for_keyframe_callback = emplaceCapture(keyframe_from_callback,
                                                                getSensorPtr(),
                                                                ts_from_callback,
                                                                Eigen::VectorXs::Zero(data_size_),
                                                                existing_capture->getDataCovariance(),
                                                                existing_capture->getCalibration(),
                                                                existing_capture->getCalibration(),
                                                                keyframe_origin);

            // split the buffer
            // and give the part of the buffer before the new keyframe to the capture for the KF callback
            existing_capture->getBuffer().split(ts_from_callback, capture_for_keyframe_callback->getBuffer());


            // interpolate individual delta
            if (!existing_capture->getBuffer().get().empty() && capture_for_keyframe_callback->getBuffer().get().back().ts_ != ts_from_callback)
            {
                // interpolate Motion at the new time stamp
                Motion motion_interpolated = interpolate(capture_for_keyframe_callback->getBuffer().get().back(), // last Motion of old buffer
                                                         existing_capture->getBuffer().get().front(), // first motion of new buffer
                                                         ts_from_callback);
                // add to old buffer
                capture_for_keyframe_callback->getBuffer().get().push_back(motion_interpolated);
            }


            // create motion feature and add it to the capture
            auto new_feature = emplaceFeature(capture_for_keyframe_callback);

            // create motion constraint and add it to the feature, and constrain to the other capture (origin)
            emplaceConstraint(new_feature, keyframe_origin->getCaptureOf(getSensorPtr()) );

            // Update the existing capture
            existing_capture->setOriginFramePtr(keyframe_from_callback);

            // re-integrate existing buffer -- note: the result of re-integration is stored in the same buffer!
            reintegrateBuffer(existing_capture);

            // modify existing feature and constraint (if they exist in the existing capture)
            if (!existing_capture->getFeatureList().empty())
            {
                auto existing_feature = existing_capture->getFeatureList().back(); // there is only one feature!

                // Modify existing feature --------
                existing_feature->setMeasurement          (existing_capture->getBuffer().get().back().delta_integr_);
                existing_feature->setMeasurementCovariance(existing_capture->getBuffer().get().back().delta_integr_cov_);

                // Modify existing constraint --------
                // Instead of modifying, we remove one ctr, and create a new one.
                auto ctr_to_remove  = existing_feature->getConstraintList().back(); // there is only one constraint!
                auto new_ctr        = emplaceConstraint(existing_feature, capture_for_keyframe_callback);
                ctr_to_remove       ->remove();  // remove old constraint now (otherwise c->remove() gets propagated to f, C, F, etc.)
            }
            break;
        }


        case RUNNING_WITH_PACK_AFTER_ORIGIN :
        {
            // extract pack elements
            FrameBasePtr keyframe_from_callback = pack->key_frame;
            TimeStamp    ts_from_callback       = keyframe_from_callback->getTimeStamp();

            // Find the frame acting as the capture's origin
            auto keyframe_origin = last_ptr_->getOriginFramePtr();

            // emplace a new motion capture to the new keyframe
            VectorXs calib = last_ptr_->getCalibration();
            auto capture_for_keyframe_callback = emplaceCapture(keyframe_from_callback,
                                                                getSensorPtr(),
                                                                ts_from_callback,
                                                                Eigen::VectorXs::Zero(data_size_),
                                                                last_ptr_->getDataCovariance(),
                                                                calib,
                                                                calib,
                                                                keyframe_origin);

            // split the buffer
            // and give the part of the buffer before the new keyframe to the capture for the KF callback
            last_ptr_->getBuffer().split(ts_from_callback, capture_for_keyframe_callback->getBuffer());

            // interpolate individual delta
            if (!last_ptr_->getBuffer().get().empty() && capture_for_keyframe_callback->getBuffer().get().back().ts_ != ts_from_callback)
            {
                // interpolate Motion at the new time stamp
                Motion motion_interpolated = interpolate(capture_for_keyframe_callback->getBuffer().get().back(), // last Motion of old buffer
                                                         last_ptr_->getBuffer().get().front(), // first motion of new buffer
                                                         ts_from_callback);
                // add to old buffer
                capture_for_keyframe_callback->getBuffer().get().push_back(motion_interpolated);
            }

            // create motion feature and add it to the capture
            auto feature_for_keyframe_callback = emplaceFeature(capture_for_keyframe_callback);

            // create motion constraint and add it to the feature, and constrain to the other capture (origin)
            emplaceConstraint(feature_for_keyframe_callback, keyframe_origin->getCaptureOf(getSensorPtr()) );

            // reset processor origin
            origin_ptr_ = capture_for_keyframe_callback;

            // Update the existing capture
            last_ptr_->setOriginFramePtr(keyframe_from_callback);

            // re-integrate existing buffer -- note: the result of re-integration is stored in the same buffer!
            reintegrateBuffer(last_ptr_);

            break;
        }



        default :
            break;
    }

    ////////////////////////////////////////////////////
    // NOW on with the received data

    // integrate data
    integrateOneStep();

    // Update state and time stamps
    last_ptr_->setTimeStamp(getCurrentTimeStamp());
    last_ptr_->getFramePtr()->setTimeStamp(getCurrentTimeStamp());
    last_ptr_->getFramePtr()->setState(getCurrentState());

    if (voteForKeyFrame() && permittedKeyFrame())
    {
        // Set the frame of last_ptr as key
        auto key_frame_ptr = last_ptr_->getFramePtr();
        key_frame_ptr->setKey();

        // create motion feature and add it to the key_capture
        auto key_feature_ptr = emplaceFeature(last_ptr_);

        // create motion constraint and link it to parent feature and other frame (which is origin's frame)
        auto ctr_ptr = emplaceConstraint(key_feature_ptr, origin_ptr_);

        // create a new frame
        auto new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME,
                                                        getCurrentState(),
                                                        getCurrentTimeStamp());
        // create a new capture
        auto new_capture_ptr = emplaceCapture(new_frame_ptr,
                                              getSensorPtr(),
                                              key_frame_ptr->getTimeStamp(),
                                              Eigen::VectorXs::Zero(data_size_),
                                              Eigen::MatrixXs::Zero(data_size_, data_size_),
                                              last_ptr_->getCalibration(),
                                              last_ptr_->getCalibration(),
                                              key_frame_ptr);
        // reset the new buffer
        new_capture_ptr->getBuffer().get().push_back( motionZero(key_frame_ptr->getTimeStamp()) ) ;

        // reset integrals
        delta_                  = deltaZero();
        delta_cov_              . setZero();
        delta_integrated_       = deltaZero();
        delta_integrated_cov_   . setZero();
        jacobian_calib_         . setZero();

        // reset derived things
        resetDerived();

        // Update pointers
        origin_ptr_     = last_ptr_;
        last_ptr_       = new_capture_ptr;

        // callback to other processors
        getProblem()->keyFrameCallback(key_frame_ptr, shared_from_this(), time_tolerance_);
    }

    resetDerived(); // TODO see where to put this

    // clear incoming just in case
    incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

    postProcess();
}


void ProcessorMotion::getState(const TimeStamp& _ts, Eigen::VectorXs& _x)
{
    CaptureMotionPtr capture_motion;
    if (origin_ptr_ && _ts >= origin_ptr_->getTimeStamp())
        // timestamp found in the current processor buffer
        capture_motion = last_ptr_;
    else
        // We need to search in previous keyframes for the capture containing a motion buffer with the queried time stamp
        capture_motion = findCaptureContainingTimeStamp(_ts);

    if (capture_motion)  // We found a CaptureMotion whose buffer contains the time stamp
    {
        // Get origin state and calibration
        VectorXs state_0          = capture_motion->getOriginFramePtr()->getState();
        CaptureBasePtr cap_orig   = capture_motion->getOriginFramePtr()->getCaptureOf(getSensorPtr());
        VectorXs calib            = cap_orig->getCalibration();

        // Get delta and correct it with new bias
        VectorXs calib_preint     = capture_motion->getBuffer().getCalibrationPreint();
        Motion   motion           = capture_motion->getBuffer().getMotion(_ts);
        
        VectorXs delta_step       = motion.jacobian_calib_ * (calib - calib_preint);
        VectorXs delta            = capture_motion->correctDelta( motion.delta_integr_, delta_step);

        // Compose on top of origin state using the buffered time stamp, not the query time stamp
        // TODO Interpolate the delta to produce a state at the query time stamp _ts
        Scalar dt = motion.ts_ - capture_motion->getBuffer().get().front().ts_; // = _ts - capture_motion->getOriginPtr()->getTimeStamp();
        statePlusDelta(state_0, delta, dt, _x);
    }
    else
    {
        // We could not find any CaptureMotion for the time stamp requested
        WOLF_ERROR("Could not find any Capture for the time stamp requested. ");
        WOLF_TRACE("Did you forget to call Problem::setPrior() in your application?")
        throw std::runtime_error("Could not find any Capture for the time stamp requested. Did you forget to call Problem::setPrior() in your application?");
    }
}

//CaptureMotionPtr ProcessorMotion::findCaptureContainingTimeStamp(const TimeStamp& _ts) const
//{
//    //std::cout << "ProcessorMotion::findCaptureContainingTimeStamp: ts = " << _ts.getSeconds() << "." << _ts.getNanoSeconds() << std::endl;
//    CaptureMotionPtr capture_ptr = last_ptr_;
//    while (capture_ptr != nullptr)
//    {
//        // capture containing motion previous than the ts found:
//        if (capture_ptr->getBuffer().get().front().ts_ < _ts)
//            return capture_ptr;
//        else
//        {
//            // go to the previous motion capture
//            if (capture_ptr == last_ptr_)
//                capture_ptr = origin_ptr_;
//            else if (capture_ptr->getOriginFramePtr() == nullptr)
//                return nullptr;
//            else
//            {
//                CaptureBasePtr capture_base_ptr = capture_ptr->getOriginFramePtr()->getCaptureOf(getSensorPtr());
//                if (capture_base_ptr == nullptr)
//                    return nullptr;
//                else
//                    capture_ptr = std::static_pointer_cast<CaptureMotion>(capture_base_ptr);
//            }
//        }
//    }
//    return capture_ptr;
//}

FrameBasePtr ProcessorMotion::setOrigin(const Eigen::VectorXs& _x_origin, const TimeStamp& _ts_origin)
{
    FrameBasePtr key_frame_ptr = getProblem()->emplaceFrame(KEY_FRAME, _x_origin, _ts_origin);
    setOrigin(key_frame_ptr);

    return key_frame_ptr;
}

void ProcessorMotion::setOrigin(FrameBasePtr _origin_frame)
{
    assert(_origin_frame && "ProcessorMotion::setOrigin: Provided frame pointer is nullptr.");
    assert(_origin_frame->getTrajectoryPtr() != nullptr
            && "ProcessorMotion::setOrigin: origin frame must be in the trajectory.");
    assert(_origin_frame->isKey() && "ProcessorMotion::setOrigin: origin frame must be KEY FRAME.");

    // -------- ORIGIN ---------
    // emplace (empty) origin Capture
    origin_ptr_ = emplaceCapture(_origin_frame,
                                 getSensorPtr(),
                                 _origin_frame->getTimeStamp(),
                                 Eigen::VectorXs::Zero(data_size_),
                                 Eigen::MatrixXs::Zero(data_size_, data_size_),
                                 getSensorPtr()->getCalibration(),
                                 getSensorPtr()->getCalibration(),
                                 nullptr);

    // ---------- LAST ----------
    // Make non-key-frame for last Capture
    auto new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME,
                                                    _origin_frame->getState(),
                                                    _origin_frame->getTimeStamp());
    // emplace (emtpy) last Capture
    last_ptr_ = emplaceCapture(new_frame_ptr,
                               getSensorPtr(),
                               _origin_frame->getTimeStamp(),
                               Eigen::VectorXs::Zero(data_size_),
                               Eigen::MatrixXs::Zero(data_size_, data_size_),
                               getSensorPtr()->getCalibration(),
                               getSensorPtr()->getCalibration(),
                               _origin_frame);

    // clear and reset buffer
    getBuffer().get().push_back(motionZero(_origin_frame->getTimeStamp()));

    // Reset integrals
    delta_                  = deltaZero();
    delta_cov_              . setZero();
    delta_integrated_       = deltaZero();
    delta_integrated_cov_   . setZero();
    jacobian_calib_         . setZero();

    // Reset derived things
    resetDerived();
}

void ProcessorMotion::integrateOneStep()
{
    // Set dt
    dt_ = updateDt();

    // get vector of parameters to calibrate
    calib_ = getBuffer().getCalibrationPreint();

    // get data and convert it to delta, and obtain also the delta covariance
    computeCurrentDelta(incoming_ptr_->getData(),
                        incoming_ptr_->getDataCovariance(),
                        calib_,
                        dt_,
                        delta_,
                        delta_cov_,
                        jacobian_delta_calib_);

    // integrate the current delta to pre-integrated measurements, and get Jacobians
    deltaPlusDelta(getBuffer().get().back().delta_integr_,
                   delta_,
                   dt_,
                   delta_integrated_,
                   jacobian_delta_preint_,
                   jacobian_delta_);

    // integrate Jacobian wrt calib
    if (calib_size_ > 0)
    {
        jacobian_calib_ = jacobian_delta_preint_ * getBuffer().get().back().jacobian_calib_ + jacobian_delta_ * jacobian_delta_calib_;
        // WOLF_TRACE("jac calib: ", jacobian_calib_.row(0));
    }

    // Integrate covariance
    delta_integrated_cov_ = jacobian_delta_preint_ * getBuffer().get().back().delta_integr_cov_ * jacobian_delta_preint_.transpose()
                          + jacobian_delta_        * delta_cov_                                 * jacobian_delta_.transpose();

    // push all into buffer
    getBuffer().get().emplace_back(incoming_ptr_->getTimeStamp(),
                                   incoming_ptr_->getData(),
                                   incoming_ptr_->getDataCovariance(),
                                   delta_,
                                   delta_cov_,
                                   delta_integrated_,
                                   delta_integrated_cov_,
                                   jacobian_delta_,
                                   jacobian_delta_preint_,
                                   jacobian_calib_);
}

void ProcessorMotion::reintegrateBuffer(CaptureMotionPtr _capture_ptr)
{
    // start with empty motion
    _capture_ptr->getBuffer().get().push_front(motionZero(_capture_ptr->getOriginFramePtr()->getTimeStamp()));

    VectorXs calib = _capture_ptr->getCalibrationPreint();

    // Iterate all the buffer
    auto motion_it = _capture_ptr->getBuffer().get().begin();
    auto prev_motion_it = motion_it;
    motion_it++;
    while (motion_it != _capture_ptr->getBuffer().get().end())
    {
        // get dt
        const Scalar dt = motion_it->ts_ - prev_motion_it->ts_;

        // re-convert data to delta with the new calibration parameters
        computeCurrentDelta(motion_it->data_,
                            motion_it->data_cov_,
                            calib,
                            dt,
                            motion_it->delta_,
                            motion_it->delta_cov_,
                            jacobian_delta_calib_);

        // integrate delta into delta_integr, and rewrite the buffer
        deltaPlusDelta(prev_motion_it->delta_integr_,
                       motion_it->delta_,
                       dt,
                       motion_it->delta_integr_,
                       motion_it->jacobian_delta_integr_,
                       motion_it->jacobian_delta_);

        // integrate Jacobian wrt calib
        if (calib_size_ > 0)
            motion_it->jacobian_calib_ = motion_it->jacobian_delta_integr_ * prev_motion_it->jacobian_calib_ + motion_it->jacobian_delta_ * jacobian_delta_calib_;

        // Integrate covariance
        motion_it->delta_integr_cov_ = motion_it->jacobian_delta_integr_ * prev_motion_it->delta_integr_cov_ * motion_it->jacobian_delta_integr_.transpose()
                                     + motion_it->jacobian_delta_        * motion_it->delta_cov_             * motion_it->jacobian_delta_.transpose();

        // advance in buffer
        motion_it++;
        prev_motion_it++;
    }
}

Motion ProcessorMotion::interpolate(const Motion& _ref, Motion& _second, TimeStamp& _ts)
{
    // Check time bounds
    assert(_ref.ts_ <= _second.ts_ && "Interpolation bounds not causal.");
    assert(_ts >= _ref.ts_    && "Interpolation time is before the _ref    motion.");
    assert(_ts <= _second.ts_ && "Interpolation time is after  the _second motion.");

    // Fraction of the time interval
    Scalar tau    = (_ts - _ref.ts_) / (_second.ts_ - _ref.ts_);

    if (tau < 0.5)
    {
        // _ts is closest to _ref
        Motion interpolated                 ( _ref );
        interpolated.ts_                    = _ts;
        interpolated.data_                  . setZero();
        interpolated.data_cov_              . setZero();
        interpolated.delta_                 = deltaZero();
        interpolated.delta_cov_             . setZero();
        interpolated.jacobian_delta_integr_ . setIdentity();
        interpolated.jacobian_delta_        . setZero();

        return interpolated;
    }
    else
    {
        // _ts is closest to _second
        Motion interpolated             ( _second );
        interpolated.ts_                    = _ts;
        _second.data_                       . setZero();
        _second.data_cov_                   . setZero();
        _second.delta_                      = deltaZero();
        _second.delta_cov_                  . setZero();
        _second.jacobian_delta_integr_      . setIdentity();
        _second.jacobian_delta_             . setZero();

        return interpolated;
    }

}

CaptureMotionPtr ProcessorMotion::findCaptureContainingTimeStamp(const TimeStamp& _ts) const
{
    // We need to search in previous keyframes for the capture containing a motion buffer with the queried time stamp
    // Note: since the buffer goes from a FK through the past until the previous KF, we need to:
    //  1. See that the KF contains a CaptureMotion
    //  2. See that the TS is smaller than the KF's TS
    //  3. See that the TS is bigger than the KF's first Motion in the CaptureMotion's buffer
    FrameBasePtr     frame          = nullptr;
    CaptureBasePtr   capture        = nullptr;
    CaptureMotionPtr capture_motion = nullptr;
    for (auto frame_rev_iter = getProblem()->getTrajectoryPtr()->getFrameList().rbegin();
            frame_rev_iter != getProblem()->getTrajectoryPtr()->getFrameList().rend();
            ++frame_rev_iter)
    {
        frame   = *frame_rev_iter;
        capture = frame->getCaptureOf(getSensorPtr());
        if (capture != nullptr)
        {
            // We found a Capture belonging to this processor's Sensor ==> it is a CaptureMotion
            capture_motion = std::static_pointer_cast<CaptureMotion>(capture);
            if (_ts >= capture_motion->getBuffer().get().front().ts_)
                // Found time stamp satisfying rule 3 above !! ==> break for loop
                break;
        }
    }
    return capture_motion;
}

CaptureMotionPtr ProcessorMotion::emplaceCapture(const FrameBasePtr& _frame_own,
                                                 const SensorBasePtr& _sensor,
                                                 const TimeStamp& _ts,
                                                 const VectorXs& _data,
                                                 const MatrixXs& _data_cov,
                                                 const VectorXs& _calib,
                                                 const VectorXs& _calib_preint,
                                                 const FrameBasePtr& _frame_origin)
{
    CaptureMotionPtr capture = createCapture(_ts,
                                             _sensor,
                                             _data,
                                             _data_cov,
                                             _frame_origin);

    capture->setCalibration(_calib);
    capture->setCalibrationPreint(_calib_preint);

    // add to wolf tree
    _frame_own->addCapture(capture);
    return capture;
}

FeatureBasePtr ProcessorMotion::emplaceFeature(CaptureMotionPtr _capture_motion)
{
    FeatureBasePtr feature = createFeature(_capture_motion);
    _capture_motion->addFeature(feature);
    return feature;
}

KFPackPtr ProcessorMotion::computeProcessingStep()
{
    if (!getProblem()->priorIsSet())
    {
        WOLF_WARN ("||*||");
        WOLF_INFO (" ... It seems you missed something!");
        WOLF_ERROR("ProcessorMotion received data before being initialized.");
        WOLF_INFO ("Did you forget to issue a Problem::setPrior()?");
        throw std::runtime_error("ProcessorMotion received data before being initialized.");
    }

    KFPackPtr pack = kf_pack_buffer_.selectPackBefore(last_ptr_, time_tolerance_);

    if (pack)
    {
        if (kf_pack_buffer_.checkTimeTolerance(pack->key_frame->getTimeStamp(), pack->time_tolerance, origin_ptr_->getTimeStamp(), time_tolerance_))
        {
            WOLF_WARN("||*||");
            WOLF_INFO(" ... It seems you missed something!");
            WOLF_ERROR("Pack's KF and origin's KF have matching time stamps (i.e. below time tolerances)");
            //            throw std::runtime_error("Pack's KF and origin's KF have matching time stamps (i.e. below time tolerances)");
            processing_step_ = RUNNING_WITH_PACK_ON_ORIGIN;
        }
        else if (pack->key_frame->getTimeStamp() < origin_ptr_->getTimeStamp() - time_tolerance_)
            processing_step_ = RUNNING_WITH_PACK_BEFORE_ORIGIN;

        else
            processing_step_ = RUNNING_WITH_PACK_AFTER_ORIGIN;

    }
    else
        processing_step_ = RUNNING_WITHOUT_PACK;

    return pack;
}

}
