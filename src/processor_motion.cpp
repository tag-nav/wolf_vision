#include "processor_motion.h"
namespace wolf
{

ProcessorMotion::ProcessorMotion(const std::string& _type,
                                 Size _state_size,
                                 Size _delta_size,
                                 Size _delta_cov_size,
                                 Size _data_size,
                                 Scalar _time_tolerance,
                                 Size _calib_size) :
        ProcessorBase(_type, _time_tolerance),
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
    status_ = IDLE;
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
    WOLF_ERROR("Process got a nullptr !");
    return;
  }

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

    incoming_ptr_ = getIncomingCaptureMotion(_incoming_ptr);

    /// @todo Anything else to do ?
    if (incoming_ptr_ == nullptr) return;

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
        FeatureBasePtr key_feature_ptr = emplaceFeature(last_ptr_);

        // create motion constraint and link it to parent feature and other frame (which is origin's frame)
        auto ctr_ptr = emplaceConstraint(key_feature_ptr, origin_ptr_);

        // create a new frame
        FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME,
                                                                getCurrentState(),
                                                                getCurrentTimeStamp());
        // create a new capture
        CaptureMotionPtr new_capture_ptr = emplaceCapture(key_frame_ptr->getTimeStamp(),
                                                          getSensorPtr(),
                                                          Eigen::VectorXs::Zero(data_size_),
                                                          Eigen::MatrixXs::Zero(data_size_, data_size_),
                                                          new_frame_ptr,
                                                          key_frame_ptr);
        // reset the new buffer
        new_capture_ptr->getBuffer().get().push_back( motionZero(key_frame_ptr->getTimeStamp()) ) ;


        // reset integrals
        delta_integrated_ = deltaZero();
        delta_integrated_cov_.setZero();
        jacobian_calib_.setZero();

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

FrameBasePtr ProcessorMotion::setOrigin(const Eigen::VectorXs& _x_origin, const TimeStamp& _ts_origin)
{
    FrameBasePtr key_frame_ptr = getProblem()->emplaceFrame(KEY_FRAME, _x_origin, _ts_origin);
    setOrigin(key_frame_ptr);

    return key_frame_ptr;
}

void ProcessorMotion::setOrigin(FrameBasePtr _origin_frame)
{

    assert(_origin_frame->getTrajectoryPtr() != nullptr
            && "ProcessorMotion::setOrigin: origin frame must be in the trajectory.");
    assert(_origin_frame->isKey() && "ProcessorMotion::setOrigin: origin frame must be KEY FRAME.");

    // emplace (empty) origin Capture
    origin_ptr_ = emplaceCapture(_origin_frame->getTimeStamp(),
                                 getSensorPtr(),
                                 Eigen::VectorXs::Zero(data_size_),
                                 Eigen::MatrixXs::Zero(data_size_, data_size_),
                                 _origin_frame,
                                 nullptr);

    // Make non-key-frame for last Capture
    FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME,
                                                           _origin_frame->getState(),
                                                           _origin_frame->getTimeStamp());
    // emplace (emtpy) last Capture
    last_ptr_ = emplaceCapture(_origin_frame->getTimeStamp(),
                               getSensorPtr(),
                               Eigen::VectorXs::Zero(data_size_),
                               Eigen::MatrixXs::Zero(data_size_, data_size_),
                               new_frame_ptr,
                               _origin_frame);

    /* Status:
     * KF --- F ---
     * o      l     i
     */

    // Reset deltas
    delta_ = deltaZero();
    delta_integrated_ = deltaZero();
    delta_cov_.setZero();
    delta_integrated_cov_.setZero();
    jacobian_calib_.setZero();

    // clear and reset buffer
    getBuffer().get().clear();
    getBuffer().setCalibrationPreint(origin_ptr_->getCalibration());
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

    // emplace a new motion capture to the new keyframe
    CaptureMotionPtr new_capture = emplaceCapture(new_ts,
                                                  getSensorPtr(),
                                                  Eigen::VectorXs::Zero(data_size_),
                                                  Eigen::MatrixXs::Zero(data_size_, data_size_),
                                                  _new_keyframe,
                                                  new_keyframe_origin);

    // split the buffer
    // and give the old buffer to the key_capture
    existing_capture->getBuffer().split(new_ts, new_capture->getBuffer());
    new_capture->getBuffer().setCalibrationPreint(new_capture->getCalibration());

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

    // create motion feature and add it to the capture
    FeatureBasePtr new_feature = emplaceFeature(new_capture);

    // create motion constraint and add it to the feature, and constrain to the other capture (origin)
    emplaceConstraint(new_feature, new_keyframe_origin->getCaptureOf(getSensorPtr())); // XXX it was new_keyframe_origin



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
        auto existing_feature = existing_capture->getFeatureList().back(); // there is only one feature!

        // Modify existing feature --------
        existing_feature->setMeasurement          (existing_capture->getBuffer().get().back().delta_integr_);
        existing_feature->setMeasurementCovariance(existing_capture->getBuffer().get().back().delta_integr_cov_);

        // Modify existing constraint --------
        // Instead of modifying, we remove one ctr, and create a new one.
        auto ctr_to_remove = existing_feature->getConstraintList().back(); // there is only one constraint!
        auto new_ctr = emplaceConstraint(existing_feature, new_capture);
        ctr_to_remove->remove();  // remove old constraint now (otherwise c->remove() gets propagated to f, C, F, etc.)
    }

    return true;
}

void ProcessorMotion::integrateOneStep()
{
    // Set dt
    updateDt();

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
        jacobian_calib_ = jacobian_delta_preint_ * getBuffer().get().back().jacobian_calib_ + jacobian_delta_ * jacobian_delta_calib_;

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

    // Iterate all the buffer
    auto motion_it = _capture_ptr->getBuffer().get().begin();
    auto prev_motion_it = motion_it;
    motion_it++;
    while (motion_it != _capture_ptr->getBuffer().get().end())
    {
        // get dt
        const Scalar dt = motion_it->ts_ - prev_motion_it->ts_;

        // re-convert data to delta with the new calibration parameters
        VectorXs calib = _capture_ptr->getBuffer().getCalibrationPreint();

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

CaptureMotionPtr ProcessorMotion::getIncomingCaptureMotion(CaptureBasePtr& _incoming_ptr)
{
  return std::static_pointer_cast<CaptureMotion>(_incoming_ptr);
}

CaptureMotionPtr ProcessorMotion::getCaptureMotionContainingTimeStamp(const TimeStamp& _ts)
{
    // We need to search in previous keyframes for the capture containing a motion buffer with the queried time stamp
    // Note: since the buffer goes from a FK through the past until the previous KF, we need to:
    //  1. See that the KF contains a CaptureMotion
    //  2. See that the TS is smaller than the KF's TS
    //  3. See that the TS is bigger than the KF's first Motion in the CaptureMotion's buffer
    FrameBasePtr frame = nullptr;
    CaptureBasePtr capture = nullptr;
    CaptureMotionPtr capture_motion = nullptr;
    for (auto frame_iter = getProblem()->getTrajectoryPtr()->getFrameList().rbegin();
            frame_iter != getProblem()->getTrajectoryPtr()->getFrameList().rend(); ++frame_iter)
    {
        frame = *frame_iter;
        capture = frame->getCaptureOf(getSensorPtr());
        if (capture != nullptr)
        {
            // We found a Capture belonging to this processor's Sensor ==> it is a CaptureMotion
            capture_motion = std::static_pointer_cast<CaptureMotion>(capture);
            if (_ts >= capture_motion->getBuffer().get().front().ts_)
                // Found time stamp satisfying rule 3 above !!
                break;
        }
    }
    return capture_motion;
}

CaptureMotionPtr ProcessorMotion::emplaceCapture(const TimeStamp& _ts,
                                                 const SensorBasePtr& _sensor,
                                                 const VectorXs& _data,
                                                 const MatrixXs& _data_cov,
                                                 const FrameBasePtr& _frame_own,
                                                 const FrameBasePtr& _frame_origin)
{
    CaptureMotionPtr capture = createCapture(_ts,
                                             _sensor,
                                             _data,
                                             _data_cov,
                                             _frame_origin);

    // calib code below might also be placed inside createCapture above
    VectorXs calib(calib_size_);
    if (origin_ptr_)
        calib = origin_ptr_->getCalibration();
    else
        calib.setZero();
    capture->setCalibration(calib);

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


}
