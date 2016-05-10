/**
 * \file processor_tracker_landmark2.cpp
 *
 *  Created on: Apr 29, 2016
 *      \author: jvallve
 */

#include "processor_tracker_landmark2.h"

namespace wolf
{

ProcessorTrackerLandmark2::ProcessorTrackerLandmark2(ProcessorType _tp, const unsigned int& _max_new_features) :
    ProcessorTracker(_tp, _max_new_features)
{
}

ProcessorTrackerLandmark2::~ProcessorTrackerLandmark2()
{
    for (auto corner : corners_last_)
        corner->destruct();
    for (auto corner : corners_incoming_)
        corner->destruct();
}

unsigned int ProcessorTrackerLandmark2::processNew(const unsigned int& _max_features)
{
    std::cout << "ProcessorTrackerLandmark2::processNew" << std::endl;
    std::cout << "\tlast correspondences: " << matches_landmark_from_last_.size() << std::endl;

    /* Rationale: A keyFrame will be created using the last Capture.
     * First, we work on this Capture to detect new Features,
     * eventually create Landmarks with them,
     * and in such case create the new Constraints feature-landmark.
     * When done, we need to track these new Features to the incoming Capture.
     * At the end, all new Features are appended to the lists of known Features in
     * the last and incoming Captures.
     */
    // We first need to populate the \b last Capture with new Features
    unsigned int n = detectNewFeatures(_max_features);
    LandmarkBaseList new_landmarks;
    for (auto new_feature_ptr : new_features_last_)
    {
        // create new landmark
        LandmarkBase* new_lmk_ptr = createLandmark(new_feature_ptr);
        new_landmarks.push_back(new_lmk_ptr);
        // create new correspondence
        matches_landmark_from_last_[new_feature_ptr] = LandmarkMatch(new_lmk_ptr, 1); // max score
    }
    std::cout << "\tlast correspondences: " << matches_landmark_from_last_.size() << std::endl;
    // Find the new landmarks in incoming_ptr_ (if it's not the same as last_ptr_)
    if (incoming_ptr_ != last_ptr_)
    {
        findLandmarks(new_landmarks, new_features_incoming_, matches_landmark_from_incoming_);

        // Append all new Features to the Capture's list of Features
        incoming_ptr_->addDownNodeList(new_features_incoming_);
    }

    // Append all new Features to the Capture's list of Features
    last_ptr_->addDownNodeList(new_features_last_);

    // Append new landmarks to the map
    getProblem()->addLandmarkList(new_landmarks);

    // return the number of new features detected in \b last
    return n;
}

unsigned int ProcessorTrackerLandmark2::processKnown()
{
    std::cout << "ProcessorTrackerLandmark2::processKnown:" << std::endl;
    std::cout << "\tlast correspondences: " << matches_landmark_from_last_.size() << std::endl;
    std::cout << "\tincoming correspondences: " << matches_landmark_from_incoming_.size() << std::endl;

    // Find landmarks in incoming_ptr_
    FeatureBaseList known_features_list_incoming;
    unsigned int found_landmarks = findLandmarks(*(getProblem()->getMapPtr()->getLandmarkListPtr()),
                                                 known_features_list_incoming, matches_landmark_from_incoming_);
    // Append found incoming features
    incoming_ptr_->addDownNodeList(known_features_list_incoming);
    std::cout << "\tlast correspondences: " << matches_landmark_from_last_.size() << std::endl;
    std::cout << "\tincoming correspondences: " << matches_landmark_from_incoming_.size() << std::endl;
    return found_landmarks;

}

} // namespace wolf
