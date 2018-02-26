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
        processing_step_(FIRST_TIME_WITHOUT_PACK),
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

void ProcessorMotion::process2(CaptureBasePtr _incoming_ptr)
{
    if (_incoming_ptr == nullptr)
    {
        WOLF_ERROR("Received capture is nullptr.");
        return;
    }

    incoming_ptr_ = std::static_pointer_cast<CaptureMotion>(_incoming_ptr);

    preProcess(); // Derived class operations

    computeProcessingStep();

    switch(processing_step_)
    {
        case FIRST_TIME_WITH_PACK :
        {
            KFPackPtr pack = selectPack( incoming_ptr_);
            setOrigin(pack->key_frame);

            // TODO process
            break;
        }
        case FIRST_TIME_WITHOUT_PACK :
        {
            // Create new KF for origin
            std::cout << "PM: make KF" << std::endl;
            VectorXs x0 = getProblem()->zeroState();
            setOrigin(x0, _incoming_ptr->getTimeStamp());
            break;
        }
        case SECOND_TIME_WITH_PACK :
        {
            KFPackPtr pack = selectPack( last_ptr_);
            break;
        }
        case SECOND_TIME_WITHOUT_PACK :
        {
            break;
        }
        case RUNNING_WITH_PACK :
        {
            KFPackPtr pack = selectPack( last_ptr_);
            break;
        }
        case RUNNING_WITHOUT_PACK :
        {
            preProcess();

            // integrate data
            integrateOneStep();

            // Update state and time stamps
            last_ptr_->setTimeStamp(incoming_ptr_->getTimeStamp());
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

                // reset processor origin to the new keyframe's capture
                origin_ptr_     = last_ptr_;
                last_ptr_       = new_capture_ptr;

                // reset derived things
                resetDerived();

                // callback to other processors
                getProblem()->keyFrameCallback(key_frame_ptr, shared_from_this(), time_tolerance_);
            }


            postProcess();

            // clear incoming just in case
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

            break;
        }
        default :
            break;
    }
}

void ProcessorMotion::process(CaptureBasePtr _incoming_ptr)
{
  if (_incoming_ptr == nullptr)
  {
    WOLF_ERROR("Received capture is nullptr.");
    return;
  }

  if ( !kf_pack_buffer_.empty() )
  {
      KFPackPtr pack;

      // Select using last_ptr
      if (last_ptr_ != nullptr)
      {
          pack = kf_pack_buffer_.selectPack( last_ptr_->getTimeStamp(), time_tolerance_ );
          if (pack!=nullptr)
          {
              keyFrameCallback(pack->key_frame,pack->time_tolerance);
              kf_pack_buffer_.removeUpTo( last_ptr_->getTimeStamp() );
          }
      }

      // Select using incoming_ptr
      pack = kf_pack_buffer_.selectPack( _incoming_ptr->getTimeStamp(), time_tolerance_ );
      if (pack!=nullptr)
      {
          keyFrameCallback(pack->key_frame,pack->time_tolerance);
          kf_pack_buffer_.removeUpTo( _incoming_ptr->getTimeStamp() );
      }
  }


  if (status_ == IDLE)
  {
      TimeStamp t0 = _incoming_ptr->getTimeStamp();

      if (origin_ptr_ == nullptr)
      {
          auto frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t0);
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
  }

  incoming_ptr_ = std::static_pointer_cast<CaptureMotion>(_incoming_ptr);

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
      auto key_frame_ptr = last_ptr_->getFramePtr();
      key_frame_ptr->setState(getCurrentState());
      key_frame_ptr->setTimeStamp(getCurrentTimeStamp());
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

      // reset processor origin to the new keyframe's capture
      origin_ptr_     = last_ptr_;
      last_ptr_       = new_capture_ptr;

      // reset derived things
      resetDerived();

      // callback to other processors
      getProblem()->keyFrameCallback(key_frame_ptr, shared_from_this(), time_tolerance_);
  }


  postProcess();

  // clear incoming just in case
  incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
}

void ProcessorMotion::getState(const TimeStamp& _ts, Eigen::VectorXs& _x)
{
    CaptureMotionPtr capture_motion;
    if (origin_ptr_ && _ts >= origin_ptr_->getTimeStamp())
        // timestamp found in the current processor buffer
        capture_motion = last_ptr_;
    else
        // We need to search in previous keyframes for the capture containing a motion buffer with the queried time stamp
        capture_motion = getCaptureMotionContainingTimeStamp(_ts);

    if (capture_motion)
    {
        // We found a CaptureMotion whose buffer contains the time stamp
        VectorXs state_0 = capture_motion->getOriginFramePtr()->getState();
        VectorXs delta = capture_motion->getDeltaCorrected(origin_ptr_->getCalibration(), _ts);
        Scalar dt = _ts - capture_motion->getBuffer().get().front().ts_;
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

CaptureMotionPtr ProcessorMotion::findCaptureContainingTimeStamp(const TimeStamp& _ts) const
{
    //std::cout << "ProcessorMotion::findCaptureContainingTimeStamp: ts = " << _ts.getSeconds() << "." << _ts.getNanoSeconds() << std::endl;
    CaptureMotionPtr capture_ptr = last_ptr_;
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

bool ProcessorMotion::keyFrameCallback(FrameBasePtr _new_keyframe, const Scalar& _time_tol_other)
{

    assert(_new_keyframe->getTrajectoryPtr() != nullptr
           && "ProcessorMotion::keyFrameCallback: key frame must be in the trajectory.");

    // get keyframe's time stamp
    TimeStamp new_ts = _new_keyframe->getTimeStamp();

    // find the capture whose buffer is affected by the new keyframe
    auto existing_capture = findCaptureContainingTimeStamp(new_ts);


    if(existing_capture == nullptr) // Keyframe without Capture --> first time
    {
        CaptureMotionPtr new_capture = createCapture(new_ts,
                                                     getSensorPtr(),
                                                     Eigen::VectorXs::Zero(data_size_),
                                                     getSensorPtr()->getNoiseCov(),
                                                     _new_keyframe);

        new_capture->setCalibration(getSensorPtr()->getCalibration());
        new_capture->setCalibrationPreint(getSensorPtr()->getCalibration());

        emplaceFrame(NON_KEY_FRAME, new_capture);
    }
    else   // Normal operation
    {

        // Find the frame acting as the capture's origin
        auto keyframe_origin = existing_capture->getOriginFramePtr();

        // emplace a new motion capture to the new keyframe
        auto new_capture = emplaceCapture(_new_keyframe,
                                          getSensorPtr(),
                                          new_ts,
                                          Eigen::VectorXs::Zero(data_size_),
                                          existing_capture->getDataCovariance(),
                                          existing_capture->getCalibration(),
                                          existing_capture->getCalibration(),
                                          keyframe_origin);

        // split the buffer
        // and give the part of the buffer before the new keyframe to the key_capture
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

        // create motion feature and add it to the capture
        auto new_feature = emplaceFeature(new_capture);
        //    reintegrateBuffer(new_capture);

        // create motion constraint and add it to the feature, and constrain to the other capture (origin)
        emplaceConstraint(new_feature, keyframe_origin->getCaptureOf(getSensorPtr()) );



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
            auto ctr_to_remove  = existing_feature->getConstraintList().back(); // there is only one constraint!
            auto new_ctr        = emplaceConstraint(existing_feature, new_capture);
            ctr_to_remove       ->remove();  // remove old constraint now (otherwise c->remove() gets propagated to f, C, F, etc.)
        }

    }

    return true;
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

KFPackPtr ProcessorMotion::selectPack(const CaptureBasePtr & _cap)
{
    if (_cap)
        return kf_pack_buffer_.selectPack(_cap->getTimeStamp(), time_tolerance_);

    return nullptr;
}

void ProcessorMotion::computeProcessingStep()
{
    // First determine the processing phase by checking the status of the tracker pointers
    enum {FIRST_TIME, SECOND_TIME, RUNNING} step;
    if (origin_ptr_ == nullptr && last_ptr_ == nullptr)
        step = FIRST_TIME;
    else if (origin_ptr_ == last_ptr_)
        step = SECOND_TIME;
    else
        step = RUNNING;


    // Then combine with the existence (or not) of a keyframe callback pack
    switch (step)
    {
        case FIRST_TIME :

            if (selectPack(incoming_ptr_))
                processing_step_ = FIRST_TIME_WITH_PACK;
            else // ! last && ! pack(incoming)
                processing_step_ = FIRST_TIME_WITHOUT_PACK;
        break;

        case SECOND_TIME :

            if (selectPack(last_ptr_))
                processing_step_ = SECOND_TIME_WITH_PACK;
            else
                processing_step_ = SECOND_TIME_WITHOUT_PACK;
            break;

        case RUNNING :
        default :

            if (selectPack(last_ptr_))
            {
                if (last_ptr_->getFramePtr()->isKey())
                {
                    WOLF_WARN("||*||");
                    WOLF_INFO(" ... It seems you missed something!");
                    WOLF_INFO("Pack's KF and last's KF have matching time stamps (i.e. below time tolerances)");
                    WOLF_INFO("Check the following for correctness:");
                    WOLF_INFO("  - You have all processors installed before starting receiving any data");
                    WOLF_INFO("  - You issued a problem->setPrior() after all processors are installed ---> ", (getProblem()->priorIsSet() ? "OK" : "NOK"));
                    WOLF_INFO("  - You have configured all your processors with compatible time tolerances");
                    WOLF_ERROR("Pack's KF and last's KF have matching time stamps (i.e. below time tolerances).");
                }
                processing_step_ = RUNNING_WITH_PACK;
            }
            else
                processing_step_ = RUNNING_WITHOUT_PACK;
            break;
    }
}

}
