/**
 * \file processor_tracker_landmark.cpp
 *
 *  Created on: Apr 7, 2016
 *      \author: jvallve
 */

#include "processor_tracker_landmark.h"
#include "map_base.h"

namespace wolf
{

ProcessorTrackerLandmark::ProcessorTrackerLandmark(const std::string& _type, const unsigned int& _max_new_features, const Scalar& _time_tolerance) :
    ProcessorTracker(_type, _max_new_features, _time_tolerance)
{
}

ProcessorTrackerLandmark::~ProcessorTrackerLandmark()
{
    for ( auto match : matches_landmark_from_incoming_)
    {
        match.second.reset(); // TODO: Should we just remove the entries? What about match.first?
    }
    for ( auto match : matches_landmark_from_last_)
    {
        match.second.reset(); // TODO: Should we just remove the entries? What about match.first?
    }
}

unsigned int ProcessorTrackerLandmark::processNew(const unsigned int& _max_features)
{
    //std::cout << "\tlast correspondences: " << matches_landmark_from_last_.size() << std::endl;
    //std::cout << "\tlast features: " << (last_ptr_ == nullptr ? 0 : last_ptr_->getFeatureList().size()) << std::endl;
    //std::cout << "\tlast new features: " << new_features_last_.size() << std::endl;
    //std::cout << "\tincoming correspondences: " << matches_landmark_from_incoming_.size() << std::endl;
    //std::cout << "\tincoming features: " << (incoming_ptr_ == nullptr ? 0 : incoming_ptr_->getFeatureList().size()) << std::endl;
    //std::cout << "\tincoming new features: " << new_features_incoming_.size() << std::endl;

    /* Rationale: A keyFrame will be created using the last Capture.
     * First, we work on this Capture to detect new Features,
     * eventually create Landmarks with them,
     * and in such case create the new Constraints feature-landmark.
     * When done, we need to track these new Features to the incoming Capture.
     * At the end, all new Features are appended to the lists of known Features in
     * the last and incoming Captures.
     */
    // We first need to populate the \b last Capture with new Features
    unsigned int n = detectNewFeatures(_max_features);
    //std::cout << "\tlast new features: " << new_features_last_.size() << std::endl;

    LandmarkBaseList new_landmarks;
    createNewLandmarks(new_landmarks);

    // Find the new landmarks in incoming_ptr_ (if it's not nullptr)
    if (incoming_ptr_ != nullptr)
    {
        findLandmarks(new_landmarks, new_features_incoming_, matches_landmark_from_incoming_);

        // Append all new Features to the Capture's list of Features
        incoming_ptr_->addFeatureList(new_features_incoming_);
    }

    // Append all new Features to the Capture's list of Features
    last_ptr_->addFeatureList(new_features_last_);
//    std::cout << "\tnew last features added " << std::endl;

    // Append new landmarks to the map
    getProblem()->addLandmarkList(new_landmarks);
//    std::cout << "\tnew landmarks added: " << getProblem()->getMapPtr()->getLandmarkList().size() << std::endl;

    //std::cout << "end of processNew:" << std::endl;
    //std::cout << "\tlast correspondences: " << matches_landmark_from_last_.size() << std::endl;
    //std::cout << "\tlast features: " << (last_ptr_ == nullptr ? 0 : last_ptr_->getFeatureList().size()) << std::endl;
    //std::cout << "\tlast new features: " << new_features_last_.size() << std::endl;
    //std::cout << "\tincoming correspondences: " << matches_landmark_from_incoming_.size() << std::endl;
    //std::cout << "\tincoming features: " << (incoming_ptr_ == nullptr ? 0 : incoming_ptr_->getFeatureList().size()) << std::endl;
    //std::cout << "\tincoming new features: " << new_features_incoming_.size() << std::endl;

    // return the number of new features detected in \b last
    return n;
}

void ProcessorTrackerLandmark::createNewLandmarks(LandmarkBaseList& _new_landmarks)
{

    for (auto new_feature_ptr : new_features_last_)
    {
        // create new landmark
        LandmarkBasePtr new_lmk_ptr = createLandmark(new_feature_ptr);
//        std::cout << "\tnew_landmark: " << new_lmk_ptr->id() << std::endl;
        _new_landmarks.push_back(new_lmk_ptr);
        // create new correspondence
        matches_landmark_from_last_[new_feature_ptr] = std::make_shared<LandmarkMatch>(new_lmk_ptr, 1); // max score
    }
//    std::cout << "\tnew_landmarks: " << _new_landmarks.size() << std::endl;
//    std::cout << "\tlast correspondences: " << matches_landmark_from_last_.size() << std::endl;
}

unsigned int ProcessorTrackerLandmark::processKnown()
{

    //std::cout << "ProcessorTrackerLandmark::processKnown:" << std::endl;
    //std::cout << "\tlast correspondences: " << matches_landmark_from_last_.size() << std::endl;
    //std::cout << "\tlast features: " << (last_ptr_ == nullptr ? 0 : last_ptr_->getFeatureList().size()) << std::endl;
    //std::cout << "\tlast new features: " << new_features_last_.size() << std::endl;
    //std::cout << "\tincoming correspondences: " << matches_landmark_from_incoming_.size() << std::endl;
    //std::cout << "\tincoming features: " << (incoming_ptr_ == nullptr ? 0 : incoming_ptr_->getFeatureList().size()) << std::endl;
    //std::cout << "\tincoming new features: " << new_features_incoming_.size() << std::endl;

    // Find landmarks in incoming_ptr_
    FeatureBaseList known_features_list_incoming;
    unsigned int found_landmarks = findLandmarks(getProblem()->getMapPtr()->getLandmarkList(),
                                                 known_features_list_incoming, matches_landmark_from_incoming_);
    // Append found incoming features
    incoming_ptr_->addFeatureList(known_features_list_incoming);

    //std::cout << "end of processKnown:" << std::endl;
    //std::cout << "\tlast correspondences: " << matches_landmark_from_last_.size() << std::endl;
    //std::cout << "\tlast features: " << (last_ptr_ == nullptr ? 0 : last_ptr_->getFeatureList().size()) << std::endl;
    //std::cout << "\tlast new features: " << new_features_last_.size() << std::endl;
    //std::cout << "\tincoming correspondences: " << matches_landmark_from_incoming_.size() << std::endl;
    //std::cout << "\tincoming features: " << (incoming_ptr_ == nullptr ? 0 : incoming_ptr_->getFeatureList().size()) << std::endl;
    //std::cout << "\tincoming new features: " << new_features_incoming_.size() << std::endl;
    return found_landmarks;

}

void ProcessorTrackerLandmark::establishConstraints()
{

    //std::cout << "\tfeatures:" << last_ptr_->getFeatureList().size() << std::endl;
    //std::cout << "\tcorrespondences: " << matches_landmark_from_last_.size() << std::endl;
    for (auto last_feature : last_ptr_->getFeatureList())
    {
        auto lmk = matches_landmark_from_last_[last_feature]->landmark_ptr_;
        ConstraintBasePtr ctr_ptr = createConstraint(last_feature,
                                                     lmk);
        if (ctr_ptr != nullptr) // constraint links
        {
            last_feature->addConstraint(ctr_ptr);
            lmk->addConstrainedBy(ctr_ptr);
        }
    }
}

} // namespace wolf
