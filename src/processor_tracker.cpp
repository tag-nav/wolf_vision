/*
 * \processor_tracker.cpp
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#include "processor_tracker.h"

ProcessorTracker::ProcessorTracker(bool _autonomous, bool _uses_landmarks) :
        ProcessorBase(),
        autonomous_(_autonomous),
        use_landmarks_(_uses_landmarks),
        origin_ptr_(nullptr),
        last_ptr_(nullptr),
        incoming_ptr_(nullptr)
{
}

ProcessorTracker::~ProcessorTracker()
{
    // WARNING: This test is not fail safe. Only class design can make it safe, by ensuring
    // at all times that whenever incoming_ptr_ is not used, it points to nullptr.
    // See both flavors of reset(), and advance().
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

    // 1. First we track the known Features and create new constraints as needed
    processKnownFeatures(_incoming_ptr);

    // 2. Then we see if we want to create a KeyFrame
    if (voteForKeyFrame())
    {
        /* Rationale: A keyFrame will be created using the last Capture. First, we work on this
         * last Capture to detect new Features, eventually create Landmarks with them,
         * and in such case create the new constraints feature-landmark. Only when done, the KeyFrame
         * is effectively created, and the tracker is reset so that its origin points
         * to the brand new KeyFrame.
         */

        // We first need to populate the Capture with new Features to create new Landmarks
        detectNewFeatures(last_ptr_);

        // Then eventually create new landmarks, with the respective feature-landmark constraints ...
        if (use_landmarks_ && new_features_list_.size() > 0)
        {
            // We'll create one Landmark for each new Feature ...
            for (FeatureBase* feature_ptr : new_features_list_)
            {
                // Create one Landmark for this Feature with the Landmark factory in this class
                LandmarkBase* lmk_ptr = createLandmark(feature_ptr);
                // Create and add one constraint between the Feature and the Landmark
                ConstraintBase* constr_ptr = createConstraint(feature_ptr, lmk_ptr);
                feature_ptr->addConstraint(constr_ptr);
            }
        } // Done with Landmark creation

        // Append all new Features to the Capture's list of Features
        last_ptr_->getFeatureListPtr()->splice(last_ptr_->getFeatureListPtr()->end(), new_features_list_);

        // Make a KeyFrame from last, and reset the tracker
        this->makeKeyFrame(last_ptr_);
        this->reset();
    }
    else
    {   // We did not create a KeyFrame:
        // Update the tracker's last and incoming pointers one step ahead
        advance();
    }
}
