/**
 * \file processor_tracker_feature_dummy.cpp
 *
 *  Created on: Apr 11, 2016
 *      \author: jvallve
 */

#include "processor_tracker_feature_dummy.h"
#include "feature_base.h"

namespace wolf
{

unsigned int ProcessorTrackerFeatureDummy::trackFeatures(const FeatureBaseList& _feature_list_in,
                                                         FeatureBaseList& _feature_list_out,
                                                         FeatureMatchMap& _feature_correspondences)
{
    WOLF_INFO("tracking " , _feature_list_in.size() , " features...");

    // loosing the track of the first 2 features
    auto features_lost = 0;
    for (auto feat_in_ptr : _feature_list_in)
    {

        if ( rand() % static_cast<int>(101) < 30 )
        {
            features_lost++;

            WOLF_INFO("track: " , feat_in_ptr->trackId() , " feature: " , feat_in_ptr->id() , " lost!");
        }
        else
        {
            FeatureBasePtr ftr(std::make_shared<FeatureBase>("POINT IMAGE", feat_in_ptr->getMeasurement(), feat_in_ptr->getMeasurementCovariance()));
            _feature_list_out.push_back(ftr);
            _feature_correspondences[_feature_list_out.back()] = std::make_shared<FeatureMatch>(FeatureMatch({feat_in_ptr,0}));

            WOLF_INFO("track: " , feat_in_ptr->trackId() , " last: " , feat_in_ptr->id() , " inc: " , ftr->id() , " !" );
        }
    }

    return _feature_list_out.size();
}

bool ProcessorTrackerFeatureDummy::voteForKeyFrame()
{
    WOLF_INFO("Nbr. of active feature tracks: " , incoming_ptr_->getFeatureList().size() );

    bool vote = incoming_ptr_->getFeatureList().size() < min_feat_for_keyframe_;

    WOLF_INFO( (vote ? "Vote ": "Do not vote ") , "for KF" );

    return incoming_ptr_->getFeatureList().size() < min_feat_for_keyframe_;
}

unsigned int ProcessorTrackerFeatureDummy::detectNewFeatures(const unsigned int& _max_features)
{
    WOLF_INFO("Detecting " , _max_features , " new features..." );

    // detecting new features
    for (unsigned int i = 1; i <= _max_features; i++)
    {
        n_feature_++;
        FeatureBasePtr ftr(std::make_shared<FeatureBase>("POINT IMAGE",
                                                         n_feature_* Eigen::Vector1s::Ones(),
                                                         Eigen::MatrixXs::Ones(1, 1)));
        new_features_last_.push_back(ftr);

        WOLF_INFO("feature " , ftr->id() , " detected!" );
    }

    WOLF_INFO(new_features_last_.size() , " features detected!");

    return new_features_last_.size();
}

ConstraintBasePtr ProcessorTrackerFeatureDummy::createConstraint(FeatureBasePtr _feature_ptr,
                                                                 FeatureBasePtr _feature_other_ptr)
{
    WOLF_INFO( "creating constraint: track " , _feature_other_ptr->trackId() , " last feature " , _feature_ptr->id()
               , " with origin feature " , _feature_other_ptr->id() );

    auto ctr = std::make_shared<ConstraintEpipolar>(_feature_ptr, _feature_other_ptr, shared_from_this());

    return ctr;
}

} // namespace wolf
