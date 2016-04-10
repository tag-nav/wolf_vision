/*
 * \processor_tracker_feature.cpp
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#include "processor_tracker_feature.h"

ProcessorTrackerFeature::ProcessorTrackerFeature(ProcessorType _tp) :
        ProcessorTracker(_tp)
{
}

ProcessorTrackerFeature::~ProcessorTrackerFeature()
{
}

unsigned int ProcessorTrackerFeature::processKnown()
{
    assert(incoming_ptr_->getFeatureListPtr()->size() == 0 && "In ProcessorTrackerFeature::processKnown(): incoming_ptr_ feature list must be empty before processKnown()");
    assert(matches_last_incoming_.size() == 0 && "In ProcessorTrackerFeature::processKnown(): match list from last to incoming must be empty before processKnown()");

    // Track features from last_ptr_ to incoming_ptr_
    trackFeatures(*(last_ptr_->getFeatureListPtr()), known_features_incoming_, matches_last_incoming_);

    // Check/correct incoming-origin correspondences
    for (auto known_incoming_feature_ptr : known_features_incoming_)
        // Check and correct the correspondence
        if (!correctFeatureDrift(known_incoming_feature_ptr, matches_last_incoming_[known_incoming_feature_ptr].feature_ptr_))
        {
            // Correspondence not confirmed -> Remove correspondence and destruct incoming feature
            matches_last_incoming_.erase(known_incoming_feature_ptr);
            known_incoming_feature_ptr->destruct();
        }

    // Append not destructed incoming features -> this empties known_features_incoming_
    incoming_ptr_->addDownNodeList(known_features_incoming_);

    return matches_last_incoming_.size();
}

unsigned int ProcessorTrackerFeature::processNew()
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

    // Track new features from last to incoming. This will append new correspondences to matches_last_incoming
    trackFeatures(new_features_last_, new_features_incoming_, matches_last_incoming_);

    // Append all new Features to the Capture's list of Features
    last_ptr_->addDownNodeList(new_features_last_); //TODO JV: is it really necessary to add all new features instead of only the tracked ones? It's easier and probably faster.
    incoming_ptr_->addDownNodeList(new_features_incoming_);

    // return the number of new features detected in \b last
    return n;
}
