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

    // Create a new non-key Frame in the Trajectory with the incoming Capture
    getTop()->createFrame(NON_KEY_FRAME, _capture_ptr->getTimeStamp());
    // Make the last Frame a KeyFrame so that it gets into the solver
    _capture_ptr->getFramePtr()->setKey();
}

void ProcessorTracker::process(CaptureBase* const _incoming_ptr)
{
    assert ( autonomous_ && "Requested process() to a non-autonomous processor.");

    // First we track the known Features and create new constraints as needed
    processKnownFeatures(_incoming_ptr);
    if (voteForKeyFrame())
    {
        // If we want a new keyframe, we first need to populate the Capture with new Features to create new Landmarks
        // Detect new Features and create new landmarks
        FeatureBaseList feat_list;
        detectNewFeatures(last_ptr_, feat_list);
        if (feat_list.size() > 0)
        {
            LandmarkBaseList lmk_list(makeLandmarks(feat_list));
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
