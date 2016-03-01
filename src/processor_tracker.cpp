/*
 * \processor_tracker.cpp
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#include "processor_tracker.h"

ProcessorTracker::ProcessorTracker(unsigned int _min_nbr_of_tracks_for_keyframe) :
        ProcessorBase(),
        origin_ptr_(nullptr),
        last_ptr_(nullptr),
        incoming_ptr_(nullptr),
        min_tracks_th_( _min_nbr_of_tracks_for_keyframe)
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
    track(_incoming_ptr);
    if (voteForKeyFrame())
    {
        // TODO: check how do we create new landmarks, etc.
        markKeyFrame();
    }
    else
        advance();
}

void ProcessorTracker::extractFeatures(CaptureBase* _capture_ptr)
{
}

void ProcessorTracker::establishConstraints(CaptureBase* _capture_ptr)
{
    track(_capture_ptr);
}
