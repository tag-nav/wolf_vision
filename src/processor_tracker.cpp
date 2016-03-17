/*
 * \processor_tracker.cpp
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#include "processor_tracker.h"

ProcessorTracker::ProcessorTracker(ProcessorType _tp, bool _autonomous, bool _uses_landmarks) :
        ProcessorBase(_tp),
        autonomous_(_autonomous),
        use_landmarks_(_uses_landmarks),
        origin_ptr_(nullptr),
        last_ptr_(nullptr),
        incoming_ptr_(nullptr)
{
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
    assert ( autonomous_ && "Requested process() to a non-autonomous processor.");

    // 1. First we track the known Features and create new constraints as needed
    processKnownFeatures();

    // 2. Then we see if we want to create a KeyFrame
    if (voteForKeyFrame())
    {
        // 2.a. Detect new Features, initialize Landmarks, create Constraints
        processNewFeatures();
        // Make KeyFrame
        makeKeyFrame(incoming_ptr_);
        // Reset the Tracker
        reset();
    }
    else
    {   // We did not create a KeyFrame:
        // 2.b. Update the tracker's last and incoming pointers one step ahead
        advance();
    }
}

unsigned int ProcessorTracker::processNewFeatures()
{
    /* Rationale: A keyFrame will be created using the incoming Capture. First, we work on this
     * Capture to detect new Features, eventually create Landmarks with them,
     * and in such case create the new Constraints feature-landmark. Only when done, the KeyFrame
     * is effectively created, and the tracker is reset so that its origin points
     * to the brand new KeyFrame.
     */
    // We first need to populate the Capture with new Features
    unsigned int n = detectNewFeatures();
    if (usesLandmarks())
    {
        for (FeatureBase* feature_ptr : new_features_list_)
        {
            // We'll create one Landmark for each new Feature ...
            LandmarkBase* lmk_ptr = createLandmark(feature_ptr);
            // Create one Constraint between the Feature and the Landmark
            ConstraintBase* constr_ptr = createConstraint(feature_ptr, lmk_ptr);
            // Add the landmark to the map --> query WolfProblem for this
            getTop()->addLandmark(lmk_ptr);
            // Add the Constraint to the Feature's constraints list
            feature_ptr->addConstraint(constr_ptr);
        }
    } // Done with Landmark creation

    processKnownFeatures();

    // Append all new Features to the Capture's list of Features
    last_ptr_->getFeatureListPtr()->splice(last_ptr_->getFeatureListPtr()->end(), new_features_list_);
    incoming_ptr_->getFeatureListPtr()->splice(incoming_ptr_->getFeatureListPtr()->end(), new_features_list_incoming_);
    return n;
}

void ProcessorTracker::makeKeyFrame(CaptureBase* _capture_ptr)
{
    assert (autonomous_ && "Requested makeKeyFrame() to a non-autonomous processor.");

    // Create a new non-key Frame in the Trajectory with the incoming Capture
    getTop()->createFrame(NON_KEY_FRAME, _capture_ptr->getTimeStamp());
    // Make the Frame a KeyFrame so that it gets into the solver
    _capture_ptr->getFramePtr()->setKey();
}

