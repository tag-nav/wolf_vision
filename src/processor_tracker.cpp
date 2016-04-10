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
