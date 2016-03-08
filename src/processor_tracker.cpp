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
        detectNewFeatures(last_ptr_);
        if (new_features_list_.size() > 0)
        {
            // Create the landmarks with the landmark factory in this class
            createLandmarks();
            // append the landmarks to the Map
            for (auto lmk_ptr : new_landmarks_list_)
            {
                getTop()->addLandmark(lmk_ptr);
            }
            clearNewLandmarksList();
        }

        // Add all new Features to the Capture
        for (FeatureBase* feat_ptr : new_features_list_)
            last_ptr_->addFeature(feat_ptr);
        clearNewFeaturesList();

        // Make a KeyFrame from last and reset the tracker
        this->makeKeyFrame(last_ptr_);
        reset();
    }
    else
        advance();
}

void ProcessorTracker::createLandmarks()
{
    clearNewLandmarksList();
    for (FeatureBase* feature_ptr : new_features_list_)
    {
        new_landmarks_list_.push_back(createOneLandmark(feature_ptr));
        // Create constraint between Feature and Landmark
        ConstraintBase* constr_ptr = createConstraint(feature_ptr, new_landmarks_list_.back());
        feature_ptr->addConstraint(constr_ptr);
    }
}
