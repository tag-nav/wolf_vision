/*
 * \processor_tracker_feature.cpp
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#include "processor_tracker_feature.h"

namespace wolf
{

ProcessorTrackerFeature::ProcessorTrackerFeature(const std::string& _type, const unsigned int _max_new_features) :
        ProcessorTracker(_type, _max_new_features)
{
}

ProcessorTrackerFeature::~ProcessorTrackerFeature()
{
}

unsigned int ProcessorTrackerFeature::processKnown()
{

//    std::cout << "ProcessorTrackerFeature::processKnown()" << std::endl;

    assert(incoming_ptr_->getFeatureList().size() == 0
            && "In ProcessorTrackerFeature::processKnown(): incoming_ptr_ feature list must be empty before processKnown()");
    assert(matches_last_from_incoming_.size() == 0
            && "In ProcessorTrackerFeature::processKnown(): match list from last to incoming must be empty before processKnown()");

    // Track features from last_ptr_ to incoming_ptr_
    trackFeatures(last_ptr_->getFeatureList(), known_features_incoming_, matches_last_from_incoming_);

//    std::cout << "Tracked: " << known_features_incoming_.size() << std::endl;

    // Check/correct incoming-origin correspondences
    if (origin_ptr_ != nullptr)
    {
        auto known_incoming_feature_it = known_features_incoming_.begin();
        while (known_incoming_feature_it != known_features_incoming_.end())
        {
            if (!correctFeatureDrift(matches_origin_from_last_[matches_last_from_incoming_[*known_incoming_feature_it]->feature_ptr_]->feature_ptr_,                                   matches_last_from_incoming_[*known_incoming_feature_it]->feature_ptr_,*known_incoming_feature_it))
            {
                // Correspondence not confirmed -> Remove correspondence and destruct incoming feature
                matches_last_from_incoming_.erase(*known_incoming_feature_it);
                // Destruct the feature
                (*known_incoming_feature_it)->remove();
                // Remove from known_features_incoming
                known_incoming_feature_it = known_features_incoming_.erase(known_incoming_feature_it);
            }
            else
                known_incoming_feature_it++;
        }
    }
    // Append not destructed incoming features -> this empties known_features_incoming_
    incoming_ptr_->addFeatureList(known_features_incoming_);
    std::cout << "Added to incoming features: " << incoming_ptr_->getFeatureList().size() << std::endl;

    return matches_last_from_incoming_.size();
}

void ProcessorTrackerFeature::advance()
{
    //    std::cout << "ProcessorTrackerFeature::advance()" << std::endl;
    // Compose correspondences to get origin_from_incoming
    for (auto match : matches_last_from_incoming_)
    {
        matches_last_from_incoming_[match.first] =
                matches_origin_from_last_[matches_last_from_incoming_[match.first]->feature_ptr_];
    }
    matches_origin_from_last_ = std::move(matches_last_from_incoming_);
    //    std::cout << "advanced correspondences: " << std::endl;
    //    std::cout << "\tincoming 2 last: " << matches_last_from_incoming_.size() << std::endl;
    //    std::cout << "\tlast 2 origin: " << std::endl;
    //    for (auto match : matches_origin_from_last_)
    //        std::cout << "\t\t" << match.first->getMeasurement() << " to " << match.second.feature_ptr_->getMeasurement() << std::endl;
}

void ProcessorTrackerFeature::reset()
{
    //    std::cout << "ProcessorTrackerFeature::reset()" << std::endl;
    // We also reset here the list of correspondences, which passes from last--incoming to origin--last.
    matches_origin_from_last_ = std::move(matches_last_from_incoming_);
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
        incoming_ptr_->addFeatureList(new_features_incoming_);
    }

    // Append all new Features to the last Captures' list of Features
    last_ptr_->addFeatureList(new_features_last_);

    // return the number of new features detected in \b last
    return n;
}

void ProcessorTrackerFeature::establishConstraints()
{
    for (auto match : matches_origin_from_last_)
    {
        auto ctr = createConstraint(match.first, match.second->feature_ptr_);
        match.first->addConstraint(ctr);
        match.second->feature_ptr_->addConstrainedBy(ctr);
    }
}

} // namespace wolf
