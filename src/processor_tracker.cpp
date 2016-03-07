/*
 * \processor_tracker.cpp
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#include "processor_tracker.h"

ProcessorTracker::ProcessorTracker(bool _autonomous) :
        ProcessorBase(),
        autonomous_(_autonomous),
        origin_ptr_(nullptr),
        last_ptr_(nullptr),
        incoming_ptr_(nullptr)
{
}

ProcessorTracker::~ProcessorTracker()
{
    delete origin_ptr_;
    delete last_ptr_;
    delete incoming_ptr_;
}

void ProcessorTracker::makeKeyFrame()
{
    if (!autonomous_)
    {
        // TODO: Add non-key Frame to Trajectory
        // Make the old Frame a KeyFrame
        getLastPtr()->getFramePtr()->setType(KEY_FRAME);
        // TODO: Point incoming_ptr_ (?) to the new non-key Frame
    }
    else
    {
        // TODO: what to do here?
    }
}

void ProcessorTracker::process(CaptureBase* const _incoming_ptr)
{
    processKnownFeatures(_incoming_ptr);
    if (autonomous_ && voteForKeyFrame())
    {
        makeKeyFrame();
        if (detectNewFeatures(origin_ptr_) > 0)
        {
            // TODO: See how to create new Landmarks
        }
    }
    else
        advance();
}
