/*
 * ProcessorTracker.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: jvallve
 */

// wolf
#include "processor_tracker.h"

// std
#include <cmath>

namespace wolf
{

ProcessorTracker::ProcessorTracker(ProcessorType _tp, const std::string& _type, const unsigned int _max_new_features, const Scalar& _time_tolerance) :
        ProcessorBase(_tp, _type, _time_tolerance), origin_ptr_(nullptr), last_ptr_(nullptr), incoming_ptr_(nullptr),
        max_new_features_(_max_new_features)
{
    //
}

ProcessorTracker::~ProcessorTracker()
{
    //
}

void ProcessorTracker::process(CaptureBasePtr const _incoming_ptr)
{
    using std::abs;

    std::cout << "-----ProcessorTracker::process():" << std::endl;

    incoming_ptr_ = _incoming_ptr;

    preProcess();
    // FIRST TIME
    if (origin_ptr_ == nullptr && last_ptr_ == nullptr)
    {
        std::cout << "FIRST TIME" << std::endl;
        //std::cout << "Features in origin: " << 0 << "; in last: " << 0 << std::endl;

        // advance
        advance();

        // advance this
        last_ptr_ = incoming_ptr_;
        incoming_ptr_ = nullptr;

        // keyframe creation on last
        FrameBasePtr closest_key_frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(last_ptr_->getTimeStamp());
        if (closest_key_frm && abs(closest_key_frm->getTimeStamp() - last_ptr_->getTimeStamp()) <= time_tolerance_)
        {
            closest_key_frm->addCapture(last_ptr_);
            closest_key_frm->setKey();
        }
        else
        {
            //makeFrame(last_ptr_, KEY_FRAME);
            makeFrame(last_ptr_, getProblem()->getStateAtTimeStamp(last_ptr_->getTimeStamp()), KEY_FRAME);
        }

        // Detect new Features, initialize Landmarks, create Constraints, ...
        processNew(max_new_features_);


        // Establish constraints from last
        establishConstraints();


    }
    // SECOND TIME or after KEY FRAME CALLBACK
    else if (origin_ptr_ == nullptr)
    {
        std::cout << "SECOND TIME or after KEY FRAME CALLBACK" << std::endl;

        // First we track the known Features
        processKnown();

        // Create a new non-key Frame in the Trajectory with the incoming Capture
        FrameBasePtr closest_key_frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(incoming_ptr_->getTimeStamp());
        if (closest_key_frm && abs(closest_key_frm->getTimeStamp() - incoming_ptr_->getTimeStamp()) <= time_tolerance_)
            closest_key_frm->addCapture(incoming_ptr_);
        else
            makeFrame(incoming_ptr_);

        // Reset the derived Tracker
        reset();

        // Reset this
        origin_ptr_ = last_ptr_;
        last_ptr_ = incoming_ptr_;
        incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

    }
    // OTHER TIMES
    else
    {
        std::cout << "OTHER TIMES" << std::endl;

        // 1. First we track the known Features and create new constraints as needed

        processKnown();

        // 2. Then we see if we want and we are allowed to create a KeyFrame
        FrameBasePtr last_key_frm = last_ptr_->getFramePtr();
        if (!last_key_frm || !last_key_frm->isKey()) // Last frame is missing, or is not key
            last_key_frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(last_ptr_->getTimeStamp());

        if (last_key_frm && abs(last_key_frm->getTimeStamp() - last_ptr_->getTimeStamp()) > time_tolerance_)
            last_key_frm = nullptr;


        if (voteForKeyFrame() && permittedKeyFrame() )//&& last_key_frm == nullptr ) // FIXME: This seems not to work!
        {
            // 2.a. We create a keyframe

            // Detect new Features, initialize Landmarks, create Constraints, ...
            processNew(max_new_features_);

            // Check if a keyframe is close enough so that we can just attach ourselves to it
            FrameBasePtr closest_key_frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(incoming_ptr_->getTimeStamp());
            if (closest_key_frm && abs(incoming_ptr_->getTimeStamp() - closest_key_frm->getTimeStamp()) < time_tolerance_)
                // Attach incoming capture to the keyframe found
                closest_key_frm->addCapture(incoming_ptr_);
            else
                // Create a new non-key Frame in the Trajectory with the incoming Capture
                makeFrame(incoming_ptr_);

            // Make the last Capture's Frame a KeyFrame
            setKeyFrame(last_ptr_);

            // Establish constraints between last and origin
            establishConstraints();

            // Reset the derived Tracker
            reset();

            // Reset this
            origin_ptr_ = last_ptr_;
            last_ptr_ = incoming_ptr_;
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
        }
        else
        {
            // 2.b. We did not create a KeyFrame:

            // advance the derived tracker
            advance();

            // Attach incoming and remove last
            last_ptr_       ->getFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's Frame
            last_ptr_       ->remove();
            incoming_ptr_   ->getFramePtr()->setTimeStamp(incoming_ptr_->getTimeStamp());

            // Advance this
            last_ptr_       = incoming_ptr_; // Incoming Capture takes the place of last Capture
            incoming_ptr_   = nullptr; // This line is not really needed, but it makes things clearer.
        }


    }
    postProcess();

    //std::cout << "-----End of process():" << std::endl;
}

bool ProcessorTracker::keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tol_other)
{
    assert((last_ptr_ == nullptr || last_ptr_->getFramePtr() != nullptr) && "ProcessorTracker::keyFrameCallback: last_ptr_ must have a frame always");
    Scalar time_tol = std::min(time_tolerance_, _time_tol_other);

    // Nothing to do if:
    //   - there is no last
    //   - last frame is already a key frame
    //   - last frame is too far in time from keyframe
    if (last_ptr_ == nullptr || last_ptr_->getFramePtr()->isKey() || std::abs(last_ptr_->getTimeStamp() - _keyframe_ptr->getTimeStamp()) > time_tol)
        return false;

    //std::cout << "ProcessorTracker::keyFrameCallback in sensor " << getSensorPtr()->id() << std::endl;

    // Capture last_ is added to the new keyframe
    FrameBasePtr last_old_frame = last_ptr_->getFramePtr();
    last_old_frame->unlinkCapture(last_ptr_);
    last_old_frame->remove();
    _keyframe_ptr->addCapture(last_ptr_);

    // Detect new Features, initialize Landmarks, create Constraints, ...
    processNew(max_new_features_);

    // Establish constraints between last and origin
    establishConstraints();

    // Set ready to go to 2nd case in process()
    origin_ptr_ = nullptr;

    return true;
}

void ProcessorTracker::setKeyFrame(CaptureBasePtr _capture_ptr)
{
    assert(_capture_ptr != nullptr && _capture_ptr->getFramePtr() != nullptr && "ProcessorTracker::setKeyFrame: null capture or capture without frame");
    if (!_capture_ptr->getFramePtr()->isKey())
    {
        // Set key
        _capture_ptr->getFramePtr()->setKey();
        // Set state to the keyframe
        _capture_ptr->getFramePtr()->setState(getProblem()->getStateAtTimeStamp(_capture_ptr->getTimeStamp()));
        // Call the new keyframe callback in order to let the other processors to establish their constraints
        getProblem()->keyFrameCallback(_capture_ptr->getFramePtr(), std::static_pointer_cast<ProcessorBase>(shared_from_this()), time_tolerance_);
    }
}


} // namespace wolf

