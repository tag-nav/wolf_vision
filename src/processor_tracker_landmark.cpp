/**
 * \file processor_tracker_landmark.cpp
 *
 *  Created on: Apr 7, 2016
 *      \author: jvallve
 */

#include "processor_tracker_landmark.h"
#include "map_base.h"

#include <utility>


namespace wolf
{

ProcessorTrackerLandmark::ProcessorTrackerLandmark(const std::string& _type,
                                                   ProcessorParamsTrackerLandmarkPtr _params_tracker_landmark) :
    ProcessorTracker(_type, _params_tracker_landmark),
    params_tracker_landmark_(_params_tracker_landmark)
{
    //
}

ProcessorTrackerLandmark::~ProcessorTrackerLandmark()
{
    //    All is shared_ptr: no need to destruct explicitly
    //
    //    for ( auto match : matches_landmark_from_incoming_)
    //    {
    //        match.second.reset(); // : Should we just remove the entries? What about match.first?
    //    }
    //    for ( auto match : matches_landmark_from_last_)
    //    {
    //        match.second.reset(); // : Should we just remove the entries? What about match.first?
    //    }
}

void ProcessorTrackerLandmark::advanceDerived()
{
    for (auto match : matches_landmark_from_last_)
    {
        match.second.reset(); // TODO: Should we just remove the entries? What about match.first?
    }
    matches_landmark_from_last_ = std::move(matches_landmark_from_incoming_);
    new_features_last_ = std::move(new_features_incoming_);
    //    for (auto match : matches_landmark_from_last_)
    //            std::cout << "\t" << match.first->id() << " to " << match.second->landmark_ptr_->id() << std::endl;
}

void ProcessorTrackerLandmark::resetDerived()
{
    //std::cout << "ProcessorTrackerLandmark::reset" << std::endl;
    for (auto match : matches_landmark_from_last_)
    {
        match.second.reset(); // TODO: Should we just remove the entries? What about match.first?
    }
    matches_landmark_from_last_ = std::move(matches_landmark_from_incoming_);
    new_features_last_ = std::move(new_features_incoming_);
    //    for (auto match : matches_landmark_from_last_)
    //            std::cout << "\t" << match.first->id() << " to " << match.second.landmark_ptr_->id() << std::endl;
}

unsigned int ProcessorTrackerLandmark::processNew(const unsigned int& _max_features)
{
    /* Rationale: A keyFrame will be created using the last Capture.
     * First, we work on this Capture to detect new Features,
     * eventually create Landmarks with them,
     * and in such case create the new Constraints feature-landmark.
     * When done, we need to track these new Features to the incoming Capture.
     * At the end, all new Features are appended to the lists of known Features in
     * the last and incoming Captures.
     */

    // We first need to populate the \b incoming Capture with new Features
    unsigned int n = detectNewFeatures(_max_features, new_features_last_);

    createNewLandmarks();

    // Find the new landmarks in incoming_ptr_ (if it's not nullptr)
    if (incoming_ptr_ != nullptr)
    {
        findLandmarks(new_landmarks_, new_features_incoming_, matches_landmark_from_incoming_);

        // Append all new Features to the Capture's list of Features
        incoming_ptr_->addFeatureList(new_features_incoming_);
    }

    // Append all new Features to the Capture's list of Features
    last_ptr_->addFeatureList(new_features_last_);

    // Append new landmarks to the map
    getProblem()->addLandmarkList(new_landmarks_);

    // return the number of new features detected in \b last
    return n;
}

void ProcessorTrackerLandmark::createNewLandmarks()
{
    // First, make sure the list is empty and will only contain new lmks
    new_landmarks_.clear();

    // Create a landmark for each new feature in last Capture:
    for (auto new_feature_ptr : new_features_last_)
    {
        // create new landmark
        LandmarkBasePtr new_lmk_ptr = createLandmark(new_feature_ptr);

        new_landmarks_.push_back(new_lmk_ptr);

        // create new correspondence
        matches_landmark_from_last_[new_feature_ptr] = std::make_shared<LandmarkMatch>(new_lmk_ptr, 1); // max score
    }
}

unsigned int ProcessorTrackerLandmark::processKnown()
{
    // Find landmarks in incoming_ptr_
    FeatureBaseList known_features_list_incoming;
    unsigned int n = findLandmarks(getProblem()->getMapPtr()->getLandmarkList(),
                                                 known_features_list_incoming, matches_landmark_from_incoming_);
    // Append found incoming features
    incoming_ptr_->addFeatureList(known_features_list_incoming);

    return n;
}

void ProcessorTrackerLandmark::establishConstraints()
{
    // Loop all features in last_ptr_
    for (auto last_feature : last_ptr_->getFeatureList())
    {
        auto lmk = matches_landmark_from_last_[last_feature]->landmark_ptr_;
        ConstraintBasePtr ctr_ptr = createConstraint(last_feature,
                                                     lmk);
        if (ctr_ptr != nullptr) // constraint links
        {
            last_feature->addConstraint(ctr_ptr);
            lmk->addConstrainedBy(ctr_ptr);
            FrameBasePtr frm = ctr_ptr->getFrameOtherPtr();
            if (frm)
                frm->addConstrainedBy(ctr_ptr);
            CaptureBasePtr cap = ctr_ptr->getCaptureOtherPtr();
            if (cap)
                cap->addConstrainedBy(ctr_ptr);
        }
    }
}

} // namespace wolf
