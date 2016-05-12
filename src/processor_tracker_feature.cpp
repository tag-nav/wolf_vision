/*
 * \processor_tracker_feature.cpp
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#include "processor_tracker_feature.h"

namespace wolf
{

ProcessorTrackerFeature::ProcessorTrackerFeature(ProcessorType _tp, const unsigned int _max_new_features) :
        ProcessorTracker(_tp, _max_new_features)
{
}

ProcessorTrackerFeature::~ProcessorTrackerFeature()
{
}

unsigned int ProcessorTrackerFeature::processKnown()
{
    std::cout << "ProcessorTrackerFeature::processKnown()" << std::endl;

    assert(incoming_ptr_->getFeatureListPtr()->size() == 0
            && "In ProcessorTrackerFeature::processKnown(): incoming_ptr_ feature list must be empty before processKnown()");
    assert(matches_last_from_incoming_.size() == 0
            && "In ProcessorTrackerFeature::processKnown(): match list from last to incoming must be empty before processKnown()");

    // Track features from last_ptr_ to incoming_ptr_
    trackFeatures(*(last_ptr_->getFeatureListPtr()), known_features_incoming_, matches_last_from_incoming_);

    std::cout << "Tracked: " << known_features_incoming_.size() << std::endl;

    // Check/correct incoming-origin correspondences
    for (auto known_incoming_feature_ptr : known_features_incoming_)
        // Check and correct the correspondence
        if (!correctFeatureDrift(known_incoming_feature_ptr,
                                 matches_last_from_incoming_[known_incoming_feature_ptr].feature_ptr_))
        {
            // Correspondence not confirmed -> Remove correspondence and destruct incoming feature
            matches_last_from_incoming_.erase(known_incoming_feature_ptr);
            known_incoming_feature_ptr->destruct();
        }

    // Append not destructed incoming features -> this empties known_features_incoming_
    incoming_ptr_->addDownNodeList(known_features_incoming_);
    std::cout << "Added to incoming features: " << incoming_ptr_->getFeatureListPtr()->size() << std::endl;

    return matches_last_from_incoming_.size();
}

unsigned int ProcessorTrackerFeature::processNew(const unsigned int& _max_new_features)
{
    //    std::cout << "ProcessorTrackerFeature::processNew()" << std::endl;

    /* Rationale: A keyFrame will be created using the last Capture.
     * First, we create the constraints from the existing Features in last,
     * because the ones that we want to detect later on will not be constrained by anyone yet.
     * Then, we work on this last Capture to detect new Features,
     * When done, we need to track these new Features to the incoming Capture.
     * At the end, all new Features are appended to the lists of known Features in
     * the last and incoming Captures.
     */

    // Populate the last Capture with new Features. The result is in new_features_last_.
    unsigned int n = detectNewFeatures(_max_new_features);

    //  std::cout << "detected " << n << " new features!" << std::endl;

    // Track new features from last to incoming. This will append new correspondences to matches_last_incoming
    if (incoming_ptr_ != last_ptr_ && incoming_ptr_ != nullptr) // we do not do it the first time.
    {
        trackFeatures(new_features_last_, new_features_incoming_, matches_last_from_incoming_);

        // Append all new Features to the incoming Captures' list of Features
        incoming_ptr_->addDownNodeList(new_features_incoming_);
    }

    // Append all new Features to the last Captures' list of Features
    last_ptr_->addDownNodeList(new_features_last_);

    // return the number of new features detected in \b last
    return n;
}

} // namespace wolf
