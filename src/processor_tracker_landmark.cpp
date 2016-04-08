/**
 * \file processor_tracker_landmark.cpp
 *
 *  Created on: Apr 7, 2016
 *      \author: jvallve
 */

#include "processor_tracker_landmark.h"

ProcessorTrackerLandmark::ProcessorTrackerLandmark(ProcessorType _tp) :
    ProcessorTracker(_tp)
{
    // TODO Auto-generated constructor stub

}

ProcessorTrackerLandmark::~ProcessorTrackerLandmark()
{
    // TODO Auto-generated destructor stub
}

unsigned int ProcessorTrackerLandmark::processNew()
{
    /* Rationale: A keyFrame will be created using the last Capture.
     * First, we work on this Capture to detect new Features,
     * eventually create Landmarks with them,
     * and in such case create the new Constraints feature-landmark.
     * When done, we need to track these new Features to the incoming Capture.
     * At the end, all new Features are appended to the lists of known Features in
     * the last and incoming Captures.
     */
    // We first need to populate the \b last Capture with new Features
    unsigned int n = detectNewFeatures();
    LandmarkBaseList new_landmarks;
    for (auto new_feature_ptr : new_features_list_last_)
    {
        // create new landmark
        LandmarkBase* new_lmk_ptr = createLandmark(new_feature_ptr);
        new_landmarks.push_back(new_lmk_ptr);
        // create new correspondence
        incoming_2_landmark_[new_feature_ptr] = LandmarkMatch(new_lmk_ptr, 1); // max score
    }
    // Find the new landmarks in incoming_ptr_
    findLandmarks(new_landmarks, new_features_list_incoming_, incoming_2_landmark_);

    // Append all new Features to the Capture's list of Features
    last_ptr_->addDownNodeList(new_features_list_last_);
    incoming_ptr_->addDownNodeList(new_features_list_incoming_);

    // Append new landmarks to the map
    getWolfProblem()->addLandmarkList(new_landmarks);
    // return the number of new features detected in \b last
    return n;
}

unsigned int ProcessorTrackerLandmark::processKnown()
{
    // the previous incoming_ is now last_
    last_2_landmark_ = incoming_2_landmark_;
    // new incoming doesn't have correspondences yet
    incoming_2_landmark_.clear();

    // Find landmarks in incoming_ptr_
    FeatureBaseList known_features_list_incoming;
    unsigned int found_landmarks = findLandmarks(*(getWolfProblem()->getMapPtr()->getLandmarkListPtr()),
                                                 known_features_list_incoming, incoming_2_landmark_);
    // Append found incoming features
    incoming_ptr_->addDownNodeList(known_features_list_incoming);
    return found_landmarks;
}
