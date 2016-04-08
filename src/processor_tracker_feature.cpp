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
    // Track features from last_ptr_ to incoming_ptr_
    FeatureCorrespondenceMap known_incoming_2_known_last;
    FeatureBaseList known_features_list_incoming;
    unsigned int tracked_features = trackFeatures(*(last_ptr_->getFeatureListPtr()), known_features_list_incoming, known_incoming_2_known_last);

    // Check/correct incoming-origin correspondences
    for (auto known_incoming_feature : known_features_list_incoming)
    {
        std::cout << known_incoming_2_known_last[known_incoming_feature].last_feature_ptr_->getConstraintListPtr()->size() << std::endl;
        assert(known_incoming_2_known_last[known_incoming_feature].last_feature_ptr_->getConstraintListPtr()->size() == 1
                && "More than 1 constraint in a feature after tracking!");

        // Check and correct the correspondence
        if (correctFeatureDrift(known_incoming_feature, known_incoming_2_known_last[known_incoming_feature].last_feature_ptr_))
        {
            // Correspondence corrected -> add traked feature to infoming capture & move constraint
            // 1. Add tracked feature to incoming capture
            incoming_ptr_->addFeature(known_incoming_feature);

            ConstraintBase* constraint_last_to_origin_ = known_incoming_2_known_last[known_incoming_feature].last_feature_ptr_->getConstraintListPtr()->front();
            // 2. Unlink constraint from last feature
            known_incoming_2_known_last[known_incoming_feature].last_feature_ptr_->unlinkDownNode(known_incoming_2_known_last[known_incoming_feature].last_feature_ptr_->getConstraintListPtr()->begin());
            // 3. Add constraint to incoming feature
            known_incoming_feature->addConstraint(constraint_last_to_origin_);
        }
        else
        {
            // Correspondence not confirmed -> Remove incoming feature & ignore correspondence
            known_incoming_feature->destruct();
        }
    }
    return tracked_features;
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
    FeatureCorrespondenceMap new_incoming_2_new_last;
    trackFeatures(new_features_list_last_, new_features_list_incoming_, new_incoming_2_new_last);

    // Create constraints in tracked new features from last to incoming
    for (auto new_feature_incoming : new_features_list_incoming_)
        new_feature_incoming->addConstraint(createConstraint(new_feature_incoming, new_incoming_2_new_last[new_feature_incoming].last_feature_ptr_));

    // Append all new Features to the Capture's list of Features
    last_ptr_->appendDownNodeList(new_features_list_last_); //TODO JV: is it really necessary to add all new features instead of only the tracked ones? It's easier and probably faster.
    incoming_ptr_->appendDownNodeList(new_features_list_incoming_);

    // return the number of new features detected in \b last
    return n;
}
