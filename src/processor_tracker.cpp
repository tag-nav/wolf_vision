/*
 * ProcessorTracker.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: jvallve
 */

#include "processor_tracker.h"

namespace wolf
{

ProcessorTracker::ProcessorTracker(ProcessorType _tp, const unsigned int _max_new_features) :
        ProcessorBase(_tp), origin_ptr_(nullptr), last_ptr_(nullptr), incoming_ptr_(nullptr),
        max_new_features_(_max_new_features)
{
    //
}

ProcessorTracker::~ProcessorTracker()
{
    if (incoming_ptr_ != nullptr && incoming_ptr_->upperNodePtr() != nullptr)
        incoming_ptr_->destruct();
}

void ProcessorTracker::process(CaptureBase* const _incoming_ptr)
{
    //    std::cout << "\nProcess..." << std::endl;

    incoming_ptr_ = _incoming_ptr;
    preProcess();
    // FIRST TIME
    if (origin_ptr_ == nullptr)
    {
        //        std::cout << "FIRST TIME" << std::endl;
        //        std::cout << "Features in origin: " << 0 << "; in last: " << 0 << std::endl;

        last_ptr_ = _incoming_ptr;
        origin_ptr_ = _incoming_ptr;

        if (last_ptr_->getFramePtr() == nullptr)
            makeFrame(last_ptr_);

        // Detect new Features, initialize Landmarks, create Constraints, ...
        processNew(max_new_features_);

        // Make the last Capture's Frame a KeyFrame so that it gets into the solver
        last_ptr_->getFramePtr()->setKey();

        // Establish constraints from last
        establishConstraints();

        // reset the derived tracker
        reset();

        // reset this: Clear incoming ptr. Origin and last are already OK.
        incoming_ptr_ = nullptr;

        //        std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;
    }
    // SECOND TIME
    else if (origin_ptr_ == last_ptr_)
    {
        //        std::cout << "SECOND TIME" << std::endl;
        //        std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;


        // 1. First we track the known Features and create new constraints as needed
        processKnown();

        // Make frame in incoming
        makeFrame(incoming_ptr_);

        // reset the derived tracker
        reset();

        // reset this: Update the tracker's last and incoming pointers one step ahead
        last_ptr_ = incoming_ptr_; // Incoming Capture takes the place of last Capture
        incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

        //        std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;
    }
    // OTHER TIMES
    else
    {
        //        std::cout << "OTHER TIMES" << std::endl;
        //
        //        std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;

        // 1. First we track the known Features and create new constraints as needed

        processKnown();

        // 2. Then we see if we want and we are allowed to create a KeyFrame
        if (!(voteForKeyFrame() && permittedKeyFrame()))
        {
            // We did not create a KeyFrame:

            // advance the derived tracker
            advance();

            // Advance this
            last_ptr_->getFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's Frame
            last_ptr_->destruct(); // TODO: JS->JV why this does not work?? Destruct now the obsolete last before reassigning a new pointer
            incoming_ptr_->getFramePtr()->setTimeStamp(incoming_ptr_->getTimeStamp());
            last_ptr_ = incoming_ptr_; // Incoming Capture takes the place of last Capture
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
        }
        else
        {
            // 2.b. Detect new Features, initialize Landmarks, create Constraints, ...
            processNew(max_new_features_);

            // Create a new non-key Frame in the Trajectory with the incoming Capture
            makeFrame(incoming_ptr_);

            // Make the last Capture's Frame a KeyFrame so that it gets into the solver
            last_ptr_->getFramePtr()->setKey();

            // Establish constraints between last and origin
            establishConstraints();

            // Call the new keyframe callback in order to let the other processors to establish their constraints
            getProblem()->keyFrameCallback(last_ptr_->getFramePtr(), (ProcessorBase*)this);

            // Reset the derived Tracker
            reset();

            // Reset this
            origin_ptr_ = last_ptr_;
            last_ptr_ = incoming_ptr_;
            incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
        }

        //        std::cout << "Features in origin: " << origin_ptr_->getFeatureListPtr()->size() << "; in last: " << last_ptr_->getFeatureListPtr()->size() << std::endl;

    }

    postProcess();
}

bool ProcessorTracker::keyFrameCallback(FrameBase* _keyframe_ptr)
{
    Scalar _dt_max = 0.1; // hardcoded for now

    // Nothing to do if:
    //   - there is no last
    //   - last hasn't frame (just a check)
    //   - last frame is already a key frame
    //   - last frame is too far in time from keyframe
    if (last_ptr_ == nullptr || last_ptr_->getFramePtr() == nullptr || last_ptr_->getFramePtr()->isKey() || std::abs(last_ptr_->getFramePtr()->getTimeStamp() - last_ptr_->getFramePtr()->getTimeStamp()) > _dt_max)
        return false;

    // Capture last_ is going to be added to the new keyframe
    // Detect new Features, initialize Landmarks, create Constraints, ...
    processNew(max_new_features_);

    // Create a new non-key Frame in the Trajectory with the incoming Capture
    makeFrame(incoming_ptr_);

    // Establish constraints between last and origin
    establishConstraints();

    // Reset the derived Tracker
    reset();

    // Reset this
    origin_ptr_ = last_ptr_;
    last_ptr_ = incoming_ptr_;
    incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.

    return true;
}

} // namespace wolf

