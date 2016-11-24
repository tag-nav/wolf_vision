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

ProcessorTracker::ProcessorTracker(const std::string& _type, const unsigned int _max_new_features, const Scalar& _time_tolerance) :
        ProcessorBase(_type, _time_tolerance), origin_ptr_(nullptr), last_ptr_(nullptr), incoming_ptr_(nullptr),
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

    incoming_ptr_ = _incoming_ptr;

    preProcess();

    // FIRST TIME
    if (origin_ptr_ == nullptr && last_ptr_ == nullptr)
    {
        WOLF_DEBUG( "FIRST TIME" );

        // advance
        advance();

        // advance this
        last_ptr_ = incoming_ptr_;
        incoming_ptr_ = nullptr;

        // keyframe creation on last
        FrameBasePtr closest_key_frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(last_ptr_->getTimeStamp());
        if (closest_key_frm && abs(closest_key_frm->getTimeStamp() - last_ptr_->getTimeStamp()) <= time_tolerance_)
        {
            // Set KF
            closest_key_frm->addCapture(last_ptr_);
            closest_key_frm->setKey();
            WOLF_DEBUG( "Last appended to existing F, set KF" , closest_key_frm->id() );
        }
        else
        {
            // Make KF
            FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(KEY_FRAME,
                                                                   getProblem()->getStateAtTimeStamp(last_ptr_->getTimeStamp()),
                                                                   last_ptr_->getTimeStamp());
            new_frame_ptr->addCapture(last_ptr_); // Add incoming Capture to the new Frame
            WOLF_DEBUG( "Last appended to new KF" , new_frame_ptr->id() );

            getProblem()->keyFrameCallback(new_frame_ptr, shared_from_this(), time_tolerance_);
        }

        // Detect new Features, initialize Landmarks, create Constraints, ...
        processNew(max_new_features_);

        // Establish constraints from last
        establishConstraints();
    }

    // SECOND TIME or after KEY FRAME CALLBACK
    else if (origin_ptr_ == nullptr)
    {
        WOLF_DEBUG("SECOND TIME or after KEY FRAME CALLBACK");

        // First we track the known Features
        processKnown();

        // Create a new non-key Frame in the Trajectory with the incoming Capture
        FrameBasePtr closest_key_frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(incoming_ptr_->getTimeStamp());
        if (closest_key_frm && abs(closest_key_frm->getTimeStamp() - incoming_ptr_->getTimeStamp()) <= time_tolerance_)
        {
            // Just append the Capture to the existing keyframe
            closest_key_frm->addCapture(incoming_ptr_);
            WOLF_DEBUG("Incoming appended to F" , closest_key_frm->id() );
        }
        else
        {
            // Create a frame to hold what will become the last Capture
            FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
            new_frame_ptr->addCapture(incoming_ptr_); // Add incoming Capture to the new Frame
            WOLF_DEBUG("Incoming in new F" , new_frame_ptr->id() );
        }

        // Reset the derived Tracker
        reset();

        // Reset this
        origin_ptr_     = last_ptr_;
        last_ptr_       = incoming_ptr_;
        incoming_ptr_   = nullptr; // This line is not really needed, but it makes things clearer.


    }

    // OTHER TIMES
    else
    {
        WOLF_DEBUG("OTHER TIMES");

        // 1. First we track the known Features and create new constraints as needed

        processKnown();

        // 2. Then we see if we want and we are allowed to create a KeyFrame
        // Three conditions to make a KF:
        //   - We vote for KF
        //   - Problem allows us to make keyframe
        //   - There is no existing KF very close to our Time Stamp <--- NOT SURE OF THIS

        FrameBasePtr closest_key_frm_to_last = last_ptr_->getFramePtr(); // start with the same last's frame
        if ( ! ( closest_key_frm_to_last && closest_key_frm_to_last->isKey() ) ) // last F is not KF
            closest_key_frm_to_last = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(last_ptr_->getTimeStamp());

        if (closest_key_frm_to_last && abs(closest_key_frm_to_last->getTimeStamp() - last_ptr_->getTimeStamp()) > time_tolerance_) // closest KF is not close enough
            closest_key_frm_to_last = nullptr;

        if ( !( (voteForKeyFrame() && permittedKeyFrame() ) || closest_key_frm_to_last ) )
        {

            // 2.a. We did not create a KeyFrame:

            // advance the derived tracker
            advance();

            // Advance this
            last_ptr_->getFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's last Frame
            last_ptr_->remove();
            incoming_ptr_->getFramePtr()->setTimeStamp(incoming_ptr_->getTimeStamp());
            last_ptr_ = incoming_ptr_; // Incoming Capture takes the place of last Capture
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

            WOLF_DEBUG("last <-- incoming");

        }
        else
        {

            // 2.b. We create a KF

            // Detect new Features, initialize Landmarks, create Constraints, ...
            processNew(max_new_features_);

            FrameBasePtr key_frm = getProblem()->getTrajectoryPtr()->closestKeyFrameToTimeStamp(incoming_ptr_->getTimeStamp());
            if ( abs(key_frm->getTimeStamp() - incoming_ptr_->getTimeStamp() ) < time_tolerance_)
            {
                // Append incoming to existing key-frame
                key_frm->addCapture(incoming_ptr_);
                WOLF_DEBUG("Incoming adhered to existing KF" , key_frm->id());
            }
            else
            {
                // Create a new non-key Frame in the Trajectory with the incoming Capture
                // Make a non-key-frame to hold incoming
                FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(NON_KEY_FRAME, incoming_ptr_->getTimeStamp());
                new_frame_ptr->addCapture(incoming_ptr_); // Add incoming Capture to the new Frame
                WOLF_DEBUG( "Incoming adhered to new F" , key_frm->id() );

                // Make the last Capture's Frame a KeyFrame
                setKeyFrame(last_ptr_);
                WOLF_DEBUG( "Set KEY to last F" , key_frm->id() );
            }

            // Establish constraints between last and origin
            establishConstraints();

            // Reset the derived Tracker
            reset();

            // Reset this
            origin_ptr_ = last_ptr_;
            last_ptr_ = incoming_ptr_;
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

}


    }
    postProcess();

    //std::cout << "-----End of process():" << std::endl;
}

bool ProcessorTracker::keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tol_other)
{
    WOLF_DEBUG( "PT: KF" , _keyframe_ptr->id() , " callback received at ts= " , _keyframe_ptr->getTimeStamp().get() );

    assert((last_ptr_ == nullptr || last_ptr_->getFramePtr() != nullptr) && "ProcessorTracker::keyFrameCallback: last_ptr_ must have a frame always");

    Scalar time_tol = std::min(time_tolerance_, _time_tol_other);


    // Nothing to do if:
    //   - there is no last
    //   - last frame is already a key frame
    //   - last frame is too far in time from keyframe
    if (last_ptr_ == nullptr || last_ptr_->getFramePtr()->isKey() || std::abs(last_ptr_->getTimeStamp() - _keyframe_ptr->getTimeStamp()) > time_tol)
    {
        WOLF_DEBUG( " --> nothing done" );
        return false;
    }

    WOLF_DEBUG( " --> appended last capture" );
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

