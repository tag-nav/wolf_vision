/*
 * ProcessorTracker.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: jvallve
 */

#include "processor_tracker.h"

ProcessorTracker::ProcessorTracker(ProcessorType _tp) :
    ProcessorBase(_tp),
    origin_ptr_(nullptr),
    last_ptr_(nullptr),
    incoming_ptr_(nullptr)
{
    //
}

ProcessorTracker::~ProcessorTracker()
{
    // FIXME: This test with nullptr is not fail safe. Only the class design can make it safe, by ensuring
    // at all times that whenever incoming_ptr_ is not used, it points to nullptr.
    // See both flavors of reset(), and advance().
    if (incoming_ptr_ != nullptr)
        delete incoming_ptr_;
}

void ProcessorTracker::process(CaptureBase* const _incoming_ptr)
{
    // 1. First we track the known Features and create new constraints as needed
    incoming_ptr_ = _incoming_ptr;
    processKnown();

    // 2. Then we see if we want and we are allowed to create a KeyFrame
    if (!(voteForKeyFrame() && permittedKeyFrame()))
    {
        // We did not create a KeyFrame:
        // 2.a. Update the tracker's last and incoming pointers one step ahead
        advance();
    }
    else
    {
        // 2.b. Detect new Features, initialize Landmarks, create Constraints, ...
        processNew();
        // Make KeyFrame
        makeKeyFrame();
        // Reset the Tracker
        reset();
    }
}

void ProcessorTracker::advance()
{
    if (last_ptr_ == origin_ptr_)        // The first time last_ptr = origin_ptr (see init() )
    {
        // We need to create the new non-key Frame to hold what will become the \b last Capture
        makeFrame(incoming_ptr_);
    }
    else
    {
        // We need to add \b incoming to the frame and remove and destruct \b last.
        last_ptr_->getFramePtr()->addCapture(incoming_ptr_); // Add incoming Capture to the tracker's Frame
        last_ptr_->destruct(); // TODO: JS->JV why this does not work?? Destruct now the obsolete last before reassigning a new pointer
        incoming_ptr_->getFramePtr()->setTimeStamp(incoming_ptr_->getTimeStamp());
    }
    last_ptr_ = incoming_ptr_; // Incoming Capture takes the place of last Capture
    incoming_ptr_ = nullptr; // This line is not really needed, but it makes things clearer.
}
