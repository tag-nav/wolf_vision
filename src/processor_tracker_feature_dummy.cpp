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
    std::cout << "tracking " << _feature_list_in.size() << " features..." << std::endl;

    // loosing the track of the first 2 features
    auto features_lost = 0;
    for (auto feat_in_ptr : _feature_list_in)
    {
        if (features_lost < 2)
        {
            features_lost++;
            std::cout << "feature " << feat_in_ptr->getMeasurement() << " lost!" << std::endl;
        }
        else
        {
            _feature_list_out.push_back(std::make_shared<FeatureBase>("POINT IMAGE", feat_in_ptr->getMeasurement(), feat_in_ptr->getMeasurementCovariance()));
            _feature_correspondences[_feature_list_out.back()] = std::make_shared<FeatureMatch>(FeatureMatch({feat_in_ptr,0}));
            std::cout << "feature " << feat_in_ptr->getMeasurement() << " tracked!" << std::endl;
        }
    }

    return _feature_list_out.size();
}

bool ProcessorTrackerFeatureDummy::voteForKeyFrame()
{
    std::cout << "N features: " << incoming_ptr_->getFeatureList().size() << std::endl;
    bool vote = incoming_ptr_->getFeatureList().size() < min_feat_for_keyframe_;
    std::cout << (vote ? "Vote ": "Not vote ") << "for KF" << std::endl;

    return incoming_ptr_->getFeatureList().size() < min_feat_for_keyframe_;
}

unsigned int ProcessorTrackerFeatureDummy::detectNewFeatures(const unsigned int& _max_features)
{
    std::cout << "Detecting " << _max_features << " new features..." << std::endl;

    // detecting new features
    for (unsigned int i = 1; i <= _max_features; i++)
    {
        n_feature_++;
        new_features_last_.push_back(
                std::make_shared<FeatureBase>("POINT IMAGE", n_feature_* Eigen::Vector1s::Ones(), Eigen::MatrixXs::Ones(1, 1)));
        //std::cout << "feature " << new_features_last_.back()->getMeasurement() << " detected!" << std::endl;
    }
    std::cout << new_features_last_.size() << " features detected!" << std::endl;

    return new_features_last_.size();
}

ConstraintBasePtr ProcessorTrackerFeatureDummy::createConstraint(FeatureBasePtr _feature_ptr,
                                                                 FeatureBasePtr _feature_other_ptr)
{
    //    std::cout << "creating constraint: last feature " << _feature_ptr->getMeasurement()
    //              << " with origin feature " << _feature_other_ptr->getMeasurement() << std::endl;
    auto ctr = std::make_shared<ConstraintEpipolar>(_feature_ptr, _feature_other_ptr, shared_from_this());
    //    _feature_ptr->addConstraint(ctr);
    //    _feature_other_ptr->addConstrainedBy(ctr);
    return ctr;
}

} // namespace wolf
