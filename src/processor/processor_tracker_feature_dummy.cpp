/**
 * \file processor_tracker_feature_dummy.cpp
 *
 *  Created on: Apr 11, 2016
 *      \author: jvallve
 */

#include "base/processor/processor_tracker_feature_dummy.h"
#include "base/feature/feature_base.h"

namespace wolf
{

unsigned int ProcessorTrackerFeatureDummy::count_ = 0;

unsigned int ProcessorTrackerFeatureDummy::trackFeatures(const FeatureBasePtrList& _features_last_in,
                                                         FeatureBasePtrList& _features_incoming_out,
                                                         FeatureMatchMap& _feature_correspondences)
{
    WOLF_INFO("tracking " , _features_last_in.size() , " features...");

    for (auto feat_in : _features_last_in)
    {
        if (++count_ % 3 == 2) // lose one every 3 tracks
        {
            WOLF_INFO("track: " , feat_in->trackId() , " feature: " , feat_in->id() , " lost");
        }
        else
        {
            FeatureBasePtr ftr(std::make_shared<FeatureBase>("POINT IMAGE", feat_in->getMeasurement(), feat_in->getMeasurementCovariance()));
            _features_incoming_out.push_back(ftr);
            _feature_correspondences[ftr] = std::make_shared<FeatureMatch>(FeatureMatch({feat_in,1.0}));

            WOLF_INFO("track: " , feat_in->trackId() , " last: " , feat_in->id() , " inc: " , ftr->id());
        }
    }

    return _features_incoming_out.size();
}

bool ProcessorTrackerFeatureDummy::voteForKeyFrame()
{
    WOLF_INFO("Nbr. of active feature tracks: " , incoming_ptr_->getFeatureList().size() );

    bool vote = incoming_ptr_->getFeatureList().size() < params_tracker_feature_->min_features_for_keyframe;

    WOLF_INFO( (vote ? "Vote ": "Do not vote ") , "for KF" );

    return vote;
}

unsigned int ProcessorTrackerFeatureDummy::detectNewFeatures(const int& _max_features, FeatureBasePtrList& _features_last_out)
{
    unsigned int max_features = _max_features;

    if (max_features == -1)
    {
        max_features = 10;
        WOLF_INFO("max_features unlimited, setting it to " , max_features);
    }
    WOLF_INFO("Detecting " , _max_features , " new features..." );

    // detecting new features
    for (unsigned int i = 0; i < max_features; i++)
    {
        n_feature_++;
        FeatureBasePtr ftr(std::make_shared<FeatureBase>("POINT IMAGE",
                                                         n_feature_* Eigen::Vector1s::Ones(),
                                                         Eigen::MatrixXs::Identity(1,1)));
        _features_last_out.push_back(ftr);

        WOLF_INFO("feature " , ftr->id() , " detected --> new track" );
    }

    WOLF_INFO(_features_last_out.size() , " features detected");

    return _features_last_out.size();
}

FactorBasePtr ProcessorTrackerFeatureDummy::createFactor(FeatureBasePtr _feature_ptr,
                                                                 FeatureBasePtr _feature_other_ptr)
{
    WOLF_INFO( "creating factor: track " , _feature_other_ptr->trackId() , " last feature " , _feature_ptr->id()
               , " with origin feature " , _feature_other_ptr->id() );

    auto ctr = std::make_shared<FactorEpipolar>(_feature_ptr, _feature_other_ptr, shared_from_this());

    return ctr;
}

} // namespace wolf
