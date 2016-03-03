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

void ProcessorTracker::process(CaptureBase* const _incoming_ptr)
{
    trackKnownFeatures(_incoming_ptr);
    if (autonomous_ && voteForKeyFrame())
    {
        makeKeyFrame();
        detectNewFeatures(origin_ptr_);
    }
    else
        advance();
}
