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
    if (incoming_ptr_ != nullptr)
        delete incoming_ptr_;
}

void ProcessorTracker::makeKeyFrame(CaptureBase* _capture_ptr)
{
    assert (autonomous_ && "Requested makeKeyFrame() to a non-autonomous processor.");

    // Create a new non-key Frame in the Trajectory
    // TODO: how to access the newly created Frame? -> do FrameBase* createFrame()
    getTop()->createFrame(NON_KEY_FRAME, last_ptr_->getTimeStamp());
    // Make the old Frame a KeyFrame so that it gets into the solver
    _capture_ptr->getFramePtr()->setKey();
}

void ProcessorTracker::process(CaptureBase* const _incoming_ptr)
{
    assert ( autonomous_ && "Requested process() to a non-autonomous processor.");

    processKnownFeatures(_incoming_ptr);
    if (voteForKeyFrame())
    {
        // Detect new Features and create new landmarks
        if (detectNewFeatures(last_ptr_) > 0)
        {
            LandmarkBaseList lmk_list(makeLandmarks());
            for (auto lmk_ptr : lmk_list)
                getTop()->getMapPtr()->addLandmark(lmk_ptr);
        }
        // Make a KeyFrame from last and reset the tracker
        this->makeKeyFrame(last_ptr_);
        reset();
    }
    else
        advance();
}
