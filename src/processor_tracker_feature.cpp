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
    // Compose correspondences to get incoming_2_origin
    for (auto incoming_feature : *(incoming_ptr_->getFeatureListPtr()))
        incoming_2_last_[incoming_feature].feature_ptr_ = last_2_origin_[incoming_2_last_[incoming_feature].feature_ptr_].feature_ptr_;
    // previously known as incoming_ is now last_
    last_2_origin_ = incoming_2_last_;
    // new incoming doesn't have correspondences yet
    incoming_2_last_.clear();

    assert(incoming_2_last_.size() == 0 && "In ProcessorTrackerFeature::processKnown(): incoming_2_last_ must be empty before processKnown()");
    assert(incoming_ptr_->getFeatureListPtr()->size() == 0 && "In ProcessorTrackerFeature::processKnown(): incoming_ptr_ feature list must be empty before processKnown()");

    // Track features from last_ptr_ to incoming_ptr_
    FeatureBaseList known_features_list_incoming;
    unsigned int tracked_features = trackFeatures(*(last_ptr_->getFeatureListPtr()), known_features_list_incoming,
                                                  incoming_2_last_);

    // Check/correct incoming-origin correspondences
    for (auto known_incoming_feature : known_features_list_incoming)
        // Check and correct the correspondence
        if (!correctFeatureDrift(known_incoming_feature, incoming_2_last_[known_incoming_feature].feature_ptr_))
        {
            // Correspondence not confirmed -> Remove correspondence and destruct incoming feature
            incoming_2_last_.erase(known_incoming_feature);
            known_incoming_feature->destruct();
        }

    // Append not destructed incoming features
    incoming_ptr_->appendDownNodeList(known_features_list_incoming);

    return known_features_list_incoming.size();
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

    // Track new features from last to incoming
    trackFeatures(new_features_list_last_, new_features_list_incoming_, incoming_2_last_);

    // Append all new Features to the Capture's list of Features
    last_ptr_->appendDownNodeList(new_features_list_last_); //TODO JV: is it really necessary to add all new features instead of only the tracked ones? It's easier and probably faster.
    incoming_ptr_->appendDownNodeList(new_features_list_incoming_);

    // return the number of new features detected in \b last
    return n;
}
