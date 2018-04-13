/*
 * \processor_tracker_feature.cpp
 *
 *  Created on: 27/02/2016
 *      \author: jsola
 */

#include "processor_tracker_feature.h"

namespace wolf
{

ProcessorTrackerFeature::ProcessorTrackerFeature(const std::string& _type, const Scalar _time_tolerance, const unsigned int _max_new_features) :
        ProcessorTracker(_type, _time_tolerance, _max_new_features)
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
    unsigned int n = detectNewFeatures(_max_new_features, new_features_last_);
    for (auto ftr : new_features_last_)
        track_matrix_.newTrack(last_ptr_, ftr);

    // Track new features from last to incoming. This will append new correspondences to matches_last_incoming
    trackFeatures(new_features_last_, new_features_incoming_, matches_last_from_incoming_);
    for (auto ftr : new_features_incoming_)
    {
        size_t trk_id_from_last = matches_last_from_incoming_[ftr]->feature_ptr_->trackId();
        track_matrix_.add(trk_id_from_last, incoming_ptr_, ftr);
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
    {
        size_t trk_id_from_last = match.second->feature_ptr_->trackId();
        track_matrix_.add(trk_id_from_last, incoming_ptr_, match.first);
    }

    // Check/correct incoming-origin correspondences
    if (origin_ptr_ != nullptr)
    {
        for (auto feature_in_incoming : known_features_incoming_)
        {
            size_t         track_id          = feature_in_incoming->trackId();
            FeatureBasePtr feature_in_last   = track_matrix_.feature(track_id, last_ptr_);
            FeatureBasePtr feature_in_origin = track_matrix_.feature(track_id, origin_ptr_);
            if (!(correctFeatureDrift(feature_in_origin, feature_in_last, feature_in_incoming)))
            {
                // Remove this feature from many places:
                matches_last_from_incoming_ .erase (feature_in_incoming); // remove match
                track_matrix_               .remove(feature_in_incoming); // remove from track matrix
                known_features_incoming_    .remove(feature_in_incoming); // remove from known features list
                feature_in_incoming        ->remove();                    // remove from wolf tree
            }
        }
    }

    // Add to wolf tree and clear
    incoming_ptr_->addFeatureList(known_features_incoming_);
    known_features_incoming_.clear();

    // Print resulting list of matches
//    for (auto match : matches_last_from_incoming_)
//        WOLF_DEBUG("Known track: ", match.first->trackId(), ", last: ", match.second->feature_ptr_->id(), ", inc: ", match.first->id());

    return matches_last_from_incoming_.size();
}

void ProcessorTrackerFeature::advanceDerived()
{
    // Reset here the list of correspondences.
    matches_last_from_incoming_.clear();

    // We set problem here because we could not determine Problem from incoming capture at the time of adding the features to incoming's feature list.
    for (auto ftr : incoming_ptr_->getFeatureList())
        ftr->setProblem(getProblem());

    // remove last from track matrix
    track_matrix_.remove(last_ptr_);
}

void ProcessorTrackerFeature::resetDerived()
{
    // Reset here the list of correspondences.
    matches_last_from_incoming_.clear();

    // Update features according to the move above.
    TrackMatches matches_origin_last = track_matrix_.matches(origin_ptr_, last_ptr_);

    for (auto const & pair_trkid_pair : matches_origin_last)
    {
        FeatureBasePtr feature_in_origin = pair_trkid_pair.second.first;
        FeatureBasePtr feature_in_last   = pair_trkid_pair.second.second;
        feature_in_last->setProblem(getProblem()); // Since these features were in incoming_, they had no Problem assigned.

        WOLF_DEBUG("Matches reset: track: ", pair_trkid_pair.first, " origin: ", feature_in_origin->id(), " last: ", feature_in_last->id());
    }
}

void ProcessorTrackerFeature::establishConstraints()
{
    TrackMatches matches_origin_last = track_matrix_.matches(origin_ptr_, last_ptr_);

    for (auto const & pair_trkid_pair : matches_origin_last)
    {
        FeatureBasePtr feature_in_origin = pair_trkid_pair.second.first;
        FeatureBasePtr feature_in_last   = pair_trkid_pair.second.second;

        auto constraint  = createConstraint(feature_in_last, feature_in_origin);
        feature_in_last  ->addConstraint(constraint);
        feature_in_origin->addConstrainedBy(constraint);

        WOLF_DEBUG( "Constraint: track: " , feature_in_last->trackId(),
                    " origin: "           , feature_in_origin->id() ,
                    " from last: "        , feature_in_last->id() );
    }
}

} // namespace wolf
