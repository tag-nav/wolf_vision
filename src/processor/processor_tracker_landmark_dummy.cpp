/**
 * \file processor_tracker_landmark_dummy.cpp
 *
 *  Created on: Apr 12, 2016
 *      \author: jvallve
 */

#include "base/processor/processor_tracker_landmark_dummy.h"
#include "base/landmark/landmark_corner_2D.h"
#include "base/factor/factor_corner_2D.h"

namespace wolf
{

ProcessorTrackerLandmarkDummy::ProcessorTrackerLandmarkDummy(ProcessorParamsTrackerLandmarkPtr _params_tracker_landmark) :
        ProcessorTrackerLandmark("TRACKER LANDMARK DUMMY", _params_tracker_landmark),
        n_feature_(0),
        landmark_idx_non_visible_(0)
{
    //

}

ProcessorTrackerLandmarkDummy::~ProcessorTrackerLandmarkDummy()
{
    //
}

unsigned int ProcessorTrackerLandmarkDummy::findLandmarks(const LandmarkBasePtrList& _landmarks_in,
                                                          FeatureBasePtrList& _features_incoming_out,
                                                          LandmarkMatchMap& _feature_landmark_correspondences)
{
    std::cout << "\tProcessorTrackerLandmarkDummy::findLandmarks"  << std::endl;
    std::cout << "\t\t"  << _landmarks_in.size() << " landmarks..." << std::endl;

    // loosing the track of the first 2 features
    auto landmarks_lost = 0;
    for (auto landmark_in_ptr : _landmarks_in)
    {
        if (landmark_in_ptr->getDescriptor(0) <= landmark_idx_non_visible_)
        {
            landmarks_lost++;
            std::cout << "\t\tlandmark " << landmark_in_ptr->getDescriptor() << " lost!" << std::endl;
        }
        else
        {
            _features_incoming_out.push_back(std::make_shared<FeatureBase>(
                    "POINT IMAGE",
                    landmark_in_ptr->getDescriptor(),
                    Eigen::MatrixXs::Identity(1,1)));
            _feature_landmark_correspondences[_features_incoming_out.back()] = std::make_shared<LandmarkMatch>(landmark_in_ptr, 1);
            std::cout << "\t\tlandmark " << landmark_in_ptr->getDescriptor() << " found!" << std::endl;
        }
    }
    return _features_incoming_out.size();
}

bool ProcessorTrackerLandmarkDummy::voteForKeyFrame()
{
    std::cout << "N features: " << incoming_ptr_->getFeatureList().size() << std::endl;
    bool vote = incoming_ptr_->getFeatureList().size() < 4;
    std::cout << (vote ? "Vote ": "Not vote ") << "for KF" << std::endl;
    return incoming_ptr_->getFeatureList().size() < 4;
}

unsigned int ProcessorTrackerLandmarkDummy::detectNewFeatures(const int& _max_features, FeatureBasePtrList& _features_last_out)
{
    std::cout << "\tProcessorTrackerLandmarkDummy::detectNewFeatures" << std::endl;

    unsigned int max_features = _max_features;

    if (max_features == -1)
    {
        max_features = 10;
        WOLF_INFO("max_features unlimited, setting it to " , max_features);
    }

    // detecting new features
    for (unsigned int i = 1; i <= max_features; i++)
    {
        n_feature_++;
        _features_last_out.push_back(std::make_shared<FeatureBase>("POINT IMAGE",
                                                                   n_feature_ * Eigen::Vector1s::Ones(),
                                                                   Eigen::MatrixXs::Ones(1, 1)));
        std::cout << "\t\tfeature " << _features_last_out.back()->getMeasurement() << " detected!" << std::endl;
    }
    return _features_last_out.size();
}

LandmarkBasePtr ProcessorTrackerLandmarkDummy::createLandmark(FeatureBasePtr _feature_ptr)
{
    //std::cout << "ProcessorTrackerLandmarkDummy::createLandmark" << std::endl;
    return std::make_shared<LandmarkCorner2D>(std::make_shared<StateBlock>(2), std::make_shared<StateBlock>(1), _feature_ptr->getMeasurement(0));
}

FactorBasePtr ProcessorTrackerLandmarkDummy::createFactor(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr)
{
    std::cout << "\tProcessorTrackerLandmarkDummy::createFactor" << std::endl;
    std::cout << "\t\tfeature " << _feature_ptr->getMeasurement() << std::endl;
    std::cout << "\t\tlandmark "<< _landmark_ptr->getDescriptor() << std::endl;
    return std::make_shared<FactorCorner2D>(_feature_ptr, std::static_pointer_cast<LandmarkCorner2D>(_landmark_ptr), shared_from_this());
}

} //namespace wolf
