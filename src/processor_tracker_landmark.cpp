/**
 * \file processor_tracker_landmark.cpp
 *
 *  Created on: Apr 7, 2016
 *      \author: jvallve
 */

#include "processor_tracker_landmark.h"

namespace wolf
{

ProcessorTrackerLandmark::ProcessorTrackerLandmark(ProcessorType _tp, const unsigned int& _max_new_features) :
    ProcessorTracker(_tp, _max_new_features)
{
}

ProcessorTrackerLandmark::~ProcessorTrackerLandmark()
{
}

unsigned int ProcessorTrackerLandmark::processNew(const unsigned int& _max_features)
{
    std::cout << "ProcessorTrackerLandmark::processNew: last correspondences: " << matches_landmark_from_last_.size() << std::endl;

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
    std::cout << n << " new features detected" << std::endl;
    LandmarkBaseList new_landmarks;
    for (auto new_feature_ptr : new_features_last_)
    {
        // create new landmark
        LandmarkBase* new_lmk_ptr = createLandmark(new_feature_ptr);
        new_landmarks.push_back(new_lmk_ptr);
        // create new correspondence
        matches_landmark_from_last_[new_feature_ptr] = LandmarkMatch(new_lmk_ptr, 1); // max score
    }
    std::cout << "landmarks created, last correspondences: " << matches_landmark_from_last_.size() << std::endl;
    // Find the new landmarks in incoming_ptr_ (if it's not the same as last_ptr_)
    if (incoming_ptr_ != last_ptr_)
    {
        std::cout << "finding landmarks..." << std::endl;
        findLandmarks(new_landmarks, new_features_incoming_, matches_landmark_from_incoming_);

        std::cout << "adding features to incoming..." << std::endl;
        // Append all new Features to the Capture's list of Features
        incoming_ptr_->addDownNodeList(new_features_incoming_);
    }

    // Append all new Features to the Capture's list of Features
    std::cout << "adding features to last..." << std::endl;
    last_ptr_->addDownNodeList(new_features_last_);

    // Append new landmarks to the map
    std::cout << "adding landmarks to map..." << std::endl;
    getWolfProblem()->addLandmarkList(new_landmarks);
    std::cout << "added!" << std::endl;
    // return the number of new features detected in \b last
    return n;
}

unsigned int ProcessorTrackerLandmark::processKnown()
{
    std::cout << "ProcessorTrackerLandmark::processKnown: last correspondences: " << matches_landmark_from_last_.size() << std::endl;
    std::cout << "incoming correspondences: " << matches_landmark_from_incoming_.size() << std::endl;

    // Find landmarks in incoming_ptr_
    FeatureBaseList known_features_list_incoming;
    unsigned int found_landmarks = findLandmarks(*(getWolfProblem()->getMapPtr()->getLandmarkListPtr()),
                                                 known_features_list_incoming, matches_landmark_from_incoming_);
    // Append found incoming features
    incoming_ptr_->addDownNodeList(known_features_list_incoming);
    std::cout << "last correspondences: " << matches_landmark_from_last_.size() << std::endl;
    std::cout << "incoming correspondences: " << matches_landmark_from_incoming_.size() << std::endl;
    return found_landmarks;

}

} // namespace wolf