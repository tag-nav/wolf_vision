/**
 * \file processor_tracker_landmark_dummy.cpp
 *
 *  Created on: Apr 12, 2016
 *      \author: jvallve
 */

#include "processor_tracker_landmark_dummy.h"
#include "landmark_corner_2D.h"
#include "constraint_corner_2D.h"

namespace wolf
{

ProcessorTrackerLandmarkDummy::ProcessorTrackerLandmarkDummy(const unsigned int& _max_new_features) :
        ProcessorTrackerLandmark(PRC_TRACKER_DUMMY, _max_new_features), n_feature_(0), landmark_idx_non_visible_(0)
{
    //

}

ProcessorTrackerLandmarkDummy::~ProcessorTrackerLandmarkDummy()
{
    //
}

unsigned int ProcessorTrackerLandmarkDummy::findLandmarks(const LandmarkBaseList& _landmark_list_in,
                                                          FeatureBaseList& _feature_list_out,
                                                          LandmarkMatchMap& _feature_landmark_correspondences)
{
    std::cout << "\tProcessorTrackerLandmarkDummy::findLandmarks"  << std::endl;
    std::cout << "\t\t"  << _landmark_list_in.size() << " landmarks..." << std::endl;

    // loosing the track of the first 2 features
    auto landmarks_lost = 0;
    for (auto landmark_in_ptr : _landmark_list_in)
    {
        if (landmark_in_ptr->getDescriptor(0) <= landmark_idx_non_visible_)
        {
            landmarks_lost++;
            std::cout << "\t\tlandmark " << landmark_in_ptr->getDescriptor() << " lost!" << std::endl;
        }
        else
        {
            _feature_list_out.push_back(
                    new FeatureBase(FEATURE_POINT_IMAGE, landmark_in_ptr->getDescriptor(), Eigen::MatrixXs::Ones(1, 1)));
            _feature_landmark_correspondences[_feature_list_out.back()] = LandmarkMatch({landmark_in_ptr, 0});
            std::cout << "\t\tlandmark " << landmark_in_ptr->getDescriptor() << " found!" << std::endl;
        }
    }
    return _feature_list_out.size();
}

bool ProcessorTrackerLandmarkDummy::voteForKeyFrame()
{
    return incoming_ptr_->getFeatureListPtr()->size() < 5;
}

unsigned int ProcessorTrackerLandmarkDummy::detectNewFeatures(const unsigned int& _max_features)
{
    std::cout << "\tProcessorTrackerLandmarkDummy::detectNewFeatures" << std::endl;

    // detecting 5 new features
    for (unsigned int i = 1; i <= _max_features; i++)
    {
        n_feature_++;
        new_features_last_.push_back(
                new FeatureBase(FEATURE_POINT_IMAGE, n_feature_ * Eigen::Vector1s::Ones(), Eigen::MatrixXs::Ones(1, 1)));
        std::cout << "\t\tfeature " << new_features_last_.back()->getMeasurement() << " detected!" << std::endl;
    }
    return new_features_last_.size();
}

LandmarkBase* ProcessorTrackerLandmarkDummy::createLandmark(FeatureBase* _feature_ptr)
{
    //std::cout << "ProcessorTrackerLandmarkDummy::createLandmark" << std::endl;
    return new LandmarkCorner2D(new StateBlock(2), new StateBlock(1), _feature_ptr->getMeasurement(0));
}

ConstraintBase* ProcessorTrackerLandmarkDummy::createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr)
{
    std::cout << "\tProcessorTrackerLandmarkDummy::createConstraint" << std::endl;
    std::cout << "\t\tfeature " << _feature_ptr->getMeasurement() << std::endl;
    std::cout << "\t\tlandmark "<< _landmark_ptr->getDescriptor() << std::endl;
    return new ConstraintCorner2D(_feature_ptr, (LandmarkCorner2D*)(_landmark_ptr));
}

} //namespace wolf
