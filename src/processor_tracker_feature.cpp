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

unsigned int ProcessorTrackerFeature::processNew(const unsigned int& _max_new_features)
{
    /* Rationale: A keyFrame will be created using the last Capture.
     * First, we work on the last Capture to detect new Features,
     * When done, we need to track these new Features to the incoming Capture.
     * At the end, all new Features are appended to the lists of known Features in
     * the last and incoming Captures.
     */

    // Populate the last Capture with new Features. The result is in new_features_last_.
    unsigned int n = detectNewFeatures(_max_new_features);
    for (auto ftr : new_features_last_)
    {
        ftr->setTrackId( ftr->id() );
        WOLF_DEBUG("Det track: ", ftr->trackId(), ", last: ", ftr->id());
    }

    // Track new features from last to incoming. This will append new correspondences to matches_last_incoming
    trackFeatures(new_features_last_, new_features_incoming_, matches_last_from_incoming_);
    for (auto ftr : new_features_incoming_)
    {
        ftr->setTrackId(matches_last_from_incoming_[ftr]->feature_ptr_->trackId());

        // Print new tracked features
        WOLF_DEBUG("New track: ", ftr->trackId(), ", last: ", matches_last_from_incoming_[ftr]->feature_ptr_->id(), ", inc: ", ftr->id());
    }

    // Append all new Features to the incoming Captures' list of Features
    incoming_ptr_->addFeatureList(new_features_incoming_);

    // Append all new Features to the last Captures' list of Features
    last_ptr_->addFeatureList(new_features_last_);

    // return the number of new features detected in \b last
    return n;
}

unsigned int ProcessorTrackerFeature::processKnown()
{
    assert(incoming_ptr_->getFeatureList().size() == 0
            && "In ProcessorTrackerFeature::processKnown(): incoming_ptr_ feature list must be empty before processKnown()");
    assert(matches_last_from_incoming_.size() == 0
            && "In ProcessorTrackerFeature::processKnown(): match list from last to incoming must be empty before processKnown()");

    // Track features from last_ptr_ to incoming_ptr_
    trackFeatures(last_ptr_->getFeatureList(), known_features_incoming_, matches_last_from_incoming_);
    for (auto match : matches_last_from_incoming_)
        match.first->setTrackId( match.second->feature_ptr_->trackId() );

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

    incoming_ptr_->addFeatureList(known_features_incoming_);
    known_features_incoming_.clear();

    // Print resulting list of matches
    for (auto match : matches_last_from_incoming_)
    {
        WOLF_DEBUG("Known track: ", match.first->trackId(), ", last: ", match.second->feature_ptr_->id(), ", inc: ", match.first->id());
    }

    return matches_last_from_incoming_.size();
}

void ProcessorTrackerFeature::advanceDerived()
{
    // Compose correspondences to get origin_from_incoming
    for (auto match : matches_last_from_incoming_)
    {
        matches_last_from_incoming_[match.first] =
                matches_origin_from_last_[matches_last_from_incoming_[match.first]->feature_ptr_];
    }
    matches_origin_from_last_ = std::move(matches_last_from_incoming_);

    // We set problem here because we could not determine Problem from incoming capture at the time of adding the features to incoming's feature list.
    for (auto ftr : incoming_ptr_->getFeatureList())
        ftr->setProblem(getProblem());

    // Print resulting list
    for (auto match: matches_origin_from_last_)
    {
        WOLF_DEBUG("Matches advanced: track: ", match.first->trackId(), "-", match.second->feature_ptr_->trackId(), " origin: ", match.second->feature_ptr_->id(), " last: ", match.first->id());
    }
}

void ProcessorTrackerFeature::resetDerived()
{
    // We also reset here the list of correspondences, which passes from last--incoming to origin--last.
    matches_origin_from_last_ = std::move(matches_last_from_incoming_);

    // Update features according to the move above.
    for (auto match: matches_origin_from_last_)
        match.first->setProblem(getProblem()); // Since these features were in incoming_, they had no Problem assigned.

    // Print resulting list
    for (auto match: matches_origin_from_last_)
    {
        WOLF_DEBUG("Matches reset: track: ", match.first->trackId(), "-", match.second->feature_ptr_->trackId(), " origin: ", match.second->feature_ptr_->id(), " last: ", match.first->id());
    }
}

void ProcessorTrackerFeature::establishConstraints()
{
    for (auto match : matches_origin_from_last_)
    {
        auto ctr = createConstraint(match.first, match.second->feature_ptr_);
        match.first->addConstraint(ctr);
        match.second->feature_ptr_->addConstrainedBy(ctr);
    }
    for (auto match : matches_origin_from_last_)
    {
        WOLF_DEBUG( "Constraint: track: " , match.second->feature_ptr_->trackId() ,
                    " origin: " , match.second->feature_ptr_->id() ,
                    " from last: " , match.first->id() );
    }
}

} // namespace wolf
