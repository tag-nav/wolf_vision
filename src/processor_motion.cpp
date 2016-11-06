#include "processor_motion.h"
namespace wolf
{

ProcessorMotion::ProcessorMotion(const std::string& _type, Size _state_size, Size _delta_size,
                                 Size _delta_cov_size, Size _data_size, const Scalar& _time_tolerance) :
        ProcessorBase(_type, _time_tolerance), x_size_(_state_size), delta_size_(_delta_size), delta_cov_size_(
                _delta_cov_size), data_size_(_data_size), origin_ptr_(), last_ptr_(), incoming_ptr_(), dt_(0.0), x_(
                _state_size), delta_(_delta_size), delta_cov_(_delta_cov_size, _delta_cov_size), delta_integrated_(
                _delta_size), delta_integrated_cov_(_delta_cov_size, _delta_cov_size), data_(_data_size), jacobian_delta_preint_(
                delta_cov_size_, delta_cov_size_), jacobian_delta_(delta_cov_size_, delta_cov_size_)
{
    //
}

ProcessorMotion::~ProcessorMotion()
{
//    std::cout << "destructed     -p-Mot" << id() << std::endl;
}

void ProcessorMotion::process(CaptureBasePtr _incoming_ptr)
{

    /* Status:
     * KF --- KF --- F ----
     *        o      l      i
     */

    incoming_ptr_ = std::static_pointer_cast<CaptureMotion>(_incoming_ptr);

    /* Status:
     * KF --- KF --- F ---- *
     *        o      l      i
     */

    preProcess();
    integrate();

    if (voteForKeyFrame() && permittedKeyFrame())
    {
        WOLF_DEBUG_HERE
        // key_capture
        CaptureMotion::Ptr key_capture_ptr = last_ptr_;
        FrameBasePtr key_frame_ptr = key_capture_ptr->getFramePtr();

        // Set the frame of key_capture as key
        key_frame_ptr->setState(getCurrentState());
        key_frame_ptr->setTimeStamp(getBuffer().get().back().ts_);
        key_frame_ptr->setKey();

        /* Status:
         * KF --- KF --- KF ---
         *        o      l      i
         */


        // create motion feature and add it to the key_capture
        FeatureBasePtr key_feature_ptr = std::make_shared<FeatureBase>(
                "MOTION",
                key_capture_ptr->getBuffer().get().back().delta_integr_,
                key_capture_ptr->getBuffer().get().back().delta_integr_cov_.determinant() > 0 ?
                        key_capture_ptr->getBuffer().get().back().delta_integr_cov_ :
                        Eigen::MatrixXs::Identity(delta_cov_size_, delta_cov_size_) * 1e-8);
        key_capture_ptr->addFeature(key_feature_ptr);

        // create motion constraint and link it to parent feature and other frame (which is origin's frame)
        auto ctr_ptr    =  createConstraint(key_feature_ptr, origin_ptr_->getFramePtr());
        key_feature_ptr -> addConstraint(ctr_ptr);
        origin_ptr_->getFramePtr() -> addConstrainedBy(ctr_ptr);

        // new last capture
        last_ptr_ = std::make_shared<CaptureMotion>(key_frame_ptr->getTimeStamp(),
                                                    getSensorPtr(),
                                                    Eigen::VectorXs::Zero(data_size_),
                                                    Eigen::MatrixXs::Zero(data_size_, data_size_),
                                                    key_frame_ptr);

        // create a new last frame
        FrameBasePtr new_frame_ptr = getProblem()->createFrame(NON_KEY_FRAME, key_frame_ptr->getState(), last_ptr_->getTimeStamp());
        new_frame_ptr->addCapture(last_ptr_); // Add Capture to the new Frame

        // reset processor origin to the new keyframe's capture
        origin_ptr_ = key_capture_ptr;
        getBuffer().get().push_back(Motion( {key_frame_ptr->getTimeStamp(), deltaZero(), deltaZero(),
                                             Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_),
                                             Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_)}));

        /* Status:
         * KF --- KF --- KF --- F ----
         *               o      l      i
         */

        delta_integrated_ = deltaZero();
        delta_integrated_cov_.setZero();

        // reset derived things
        resetDerived();
        getProblem()->keyFrameCallback(key_frame_ptr, shared_from_this(), time_tolerance_);

    }


    postProcess();

    // clear incoming just in case
    incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
}

CaptureMotion::Ptr ProcessorMotion::findCaptureContainingTimeStamp(const TimeStamp& _ts) const
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

void ProcessorMotion::setOrigin(FrameBasePtr _origin_frame)
{

    assert(_origin_frame->getTrajectoryPtr() != nullptr
            && "ProcessorMotion::setOrigin: origin frame must be in the trajectory.");
    assert(_origin_frame->isKey() && "ProcessorMotion::setOrigin: origin frame must be KEY FRAME.");

    /* Status:
     * * --- * ---
     * o     l     i
     */

    // make (empty) origin Capture
    origin_ptr_ = std::make_shared<CaptureMotion>(_origin_frame->getTimeStamp(), getSensorPtr(),
                                                  Eigen::VectorXs::Zero(data_size_),
                                                  Eigen::MatrixXs::Zero(data_size_, data_size_), nullptr);
    // Add origin capture to origin frame
    _origin_frame->addCapture(origin_ptr_);

    /* Status:
     * KF --- * ---
     * o      l     i
     */

    // make (emtpy) last Capture
    last_ptr_ = std::make_shared<CaptureMotion>(_origin_frame->getTimeStamp(),
                                                getSensorPtr(),
                                                Eigen::VectorXs::Zero(data_size_),
                                                Eigen::MatrixXs::Zero(data_size_, data_size_),
                                                _origin_frame);
    // Make non-key-frame at last Capture
    //    makeFrame(last_ptr_, _origin_frame->getState(), NON_KEY_FRAME);
    FrameBasePtr new_frame_ptr = getProblem()->createFrame(NON_KEY_FRAME,
                                                           _origin_frame->getState(),
                                                           last_ptr_->getTimeStamp());
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

bool ProcessorMotion::keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tol_other)
{

    //    Scalar time_tol = std::min(time_tolerance_, _time_tol_other);
    //    std::cout << "  Time tol this  " << time_tolerance_ << std::endl;
    //    std::cout << "  Time tol other " << _time_tol_other << std::endl;
    //    std::cout << "  Time tol eff   " << time_tol << std::endl;
    //
    //    std::cout << "  Time stamp input F " << _keyframe_ptr->getTimeStamp().get() << std::endl;
    //    std::cout << "  Time stamp orig  F " << getOriginPtr()->getFramePtr()->getTimeStamp().get() << std::endl;
    //    std::cout << "  Time stamp orig  C " << getOriginPtr()->getTimeStamp().get() << std::endl;
    //    std::cout << "  Time stamp last  F " << getLastPtr()->getFramePtr()->getTimeStamp().get() << std::endl;
    //    std::cout << "  Time stamp last  C " << getLastPtr()->getTimeStamp().get() << std::endl;

    WOLF_DEBUG_HERE
    std::cout << "PM: KF" << _keyframe_ptr->id() << " callback received at ts= " << _keyframe_ptr->getTimeStamp().get() << std::endl;
    std::cout << "while last ts= " << last_ptr_->getTimeStamp().get() << std::endl;
    std::cout << "while last's frame ts= " << last_ptr_->getFramePtr()->getTimeStamp().get() << std::endl;

    assert(_keyframe_ptr->getTrajectoryPtr() != nullptr
            && "ProcessorMotion::keyFrameCallback: key frame must be in the trajectory.");

    // get time stamp
    TimeStamp ts = _keyframe_ptr->getTimeStamp();

    // find capture in which the new keyframe is interpolated
    CaptureMotion::Ptr capture_ptr = findCaptureContainingTimeStamp(ts);
    assert(capture_ptr != nullptr
            && "ProcessorMotion::keyFrameCallback: no motion capture containing the required TimeStamp found");
    FrameBasePtr key_frame_origin = capture_ptr->getOriginFramePtr();

    // create motion capture
    CaptureMotion::Ptr key_capture_ptr = std::make_shared<CaptureMotion>(ts, getSensorPtr(),
                                                                         Eigen::VectorXs::Zero(data_size_),
                                                                         Eigen::MatrixXs::Zero(data_size_, data_size_),
                                                                         key_frame_origin);
    // add motion capture to keyframe
    _keyframe_ptr->addCapture(key_capture_ptr);

    // split the buffer
    // and give old buffer to new key capture
    capture_ptr->getBuffer().split(ts, key_capture_ptr->getBuffer());
    // interpolate individual delta
    Motion mot = interpolate(key_capture_ptr->getBuffer().get().back(), // last Motion of old buffer
            capture_ptr->getBuffer().get().front(), // first motion of new buffer
            ts);
    // add to old buffer
    key_capture_ptr->getBuffer().get().push_back(mot);

    // create motion feature and add it to the capture
    FeatureBasePtr key_feature_ptr = std::make_shared<FeatureBase>(
            "MOTION",
            key_capture_ptr->getBuffer().get().back().delta_integr_,
            key_capture_ptr->getBuffer().get().back().delta_integr_cov_.determinant() > 0 ?
                    key_capture_ptr->getBuffer().get().back().delta_integr_cov_ :
                    Eigen::MatrixXs::Identity(delta_size_, delta_size_) * 1e-8);
    key_capture_ptr->addFeature(key_feature_ptr);

    // create motion constraint and add it to the feature, and link it to the other frame (origin)
    auto key_ctr_ptr = createConstraint(key_feature_ptr, key_frame_origin);
    key_feature_ptr->addConstraint(key_ctr_ptr);
    key_frame_origin->addConstrainedBy(key_ctr_ptr);

    // Fix the remaining capture
    if (capture_ptr == last_ptr_)
        // reset processor origin
        origin_ptr_ = key_capture_ptr;

    capture_ptr->setOriginFramePtr(_keyframe_ptr);

    // reintegrate own buffer // XXX: where is the result of re-integration stored?
    reintegrate(capture_ptr);

    // modify feature and constraint (if they exist)
    if (!capture_ptr->getFeatureList().empty())
    {
        FeatureBasePtr feature_ptr = capture_ptr->getFeatureList().back(); // there is only one feature!

        // modify feature
        feature_ptr->setMeasurement(capture_ptr->getBuffer().get().back().delta_integr_);
        feature_ptr->setMeasurementCovariance(
                capture_ptr->getBuffer().get().back().delta_integr_cov_.determinant() > 0 ?
                        capture_ptr->getBuffer().get().back().delta_integr_cov_ :
                        Eigen::MatrixXs::Identity(delta_size_, delta_size_) * 1e-8);

        // modify constraint
        // Instead of modifying, we remove one ctr, and create a new one.

        // get the constraint to be removed later
        auto ctr_to_remove = feature_ptr->getConstraintList().back(); // there is only one constraint!

        // create and append new constraint
        auto new_ctr = feature_ptr->addConstraint(createConstraint(feature_ptr, _keyframe_ptr));
        _keyframe_ptr->addConstrainedBy(new_ctr);

        // remove old constraint now (otherwise c->remove() gets propagated to f, C, F, etc.)
        ctr_to_remove->remove();

    }

    return true;
}

void ProcessorMotion::integrate()
{

    // Set dt
    updateDt();

    // get data and convert it to delta, and obtain also the delta covariance
    data2delta(incoming_ptr_->getData(), incoming_ptr_->getDataCovariance(), dt_);

    // then integrate the current delta to pre-integrated measurements, and get Jacobians
    deltaPlusDelta(delta_integrated_, delta_, dt_, delta_integrated_, jacobian_delta_preint_, jacobian_delta_);

    // and covariance
    delta_integrated_cov_ = jacobian_delta_preint_ * getBuffer().get().back().delta_integr_cov_
            * jacobian_delta_preint_.transpose() + jacobian_delta_ * delta_cov_ * jacobian_delta_.transpose();

    // then push it into buffer
    getBuffer().get().push_back(Motion( {incoming_ptr_->getTimeStamp(), delta_, delta_integrated_, delta_cov_,
                                         delta_integrated_cov_}));

    // Update state and time stamps
    last_ptr_->getFramePtr()->setState(getCurrentState());
    last_ptr_->setTimeStamp(incoming_ptr_->getTimeStamp());
    last_ptr_->getFramePtr()->setTimeStamp(last_ptr_->getTimeStamp());
}

void ProcessorMotion::reintegrate(CaptureMotion::Ptr _capture_ptr)
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

        // integrate delta into delta_integr
        deltaPlusDelta(prev_motion_it->delta_integr_, motion_it->delta_, dt, motion_it->delta_integr_,
                       jacobian_delta_preint_, jacobian_delta_);

        // integrate covariances
        delta_integrated_cov_ = jacobian_delta_preint_ * getBuffer().get().back().delta_integr_cov_
                * jacobian_delta_preint_.transpose() + jacobian_delta_ * delta_cov_ * jacobian_delta_.transpose();

        // XXX: Are we not pushing into buffer?

        // advance in buffer
        motion_it++;
        prev_motion_it++;
    }
}

}
