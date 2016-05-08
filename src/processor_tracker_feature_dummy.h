/**
 * \file processor_tracker_feature_dummy.h
 *
 *  Created on: Apr 11, 2016
 *      \author: jvallve
 */

#ifndef PROCESSOR_TRACKER_FEATURE_DUMMY_H_
#define PROCESSOR_TRACKER_FEATURE_DUMMY_H_

#include "wolf.h"
#include "processor_tracker_feature.h"
#include "constraint_epipolar.h"

namespace wolf
{

class ProcessorTrackerFeatureDummy : public wolf::ProcessorTrackerFeature
{

    public:
        ProcessorTrackerFeatureDummy();
        virtual ~ProcessorTrackerFeatureDummy();

    protected:

        unsigned int n_feature_;

//        virtual void preProcess() { }
//        virtual void postProcess() { }

        /** \brief Track provided features from \b last to \b incoming
         * \param _feature_list_in input list of features in \b last to track
         * \param _feature_list_out returned list of features found in \b incoming
         * \param _feature_correspondences returned map of correspondences: _feature_correspondences[feature_out_ptr] = feature_in_ptr
         */
        virtual unsigned int trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out,
                                           FeatureMatchMap& _feature_correspondences);

        /** \brief Correct the drift in incoming feature by re-comparing against the corresponding feature in origin.
         * \param _last_feature input feature in last capture tracked
         * \param _incoming_feature input/output feature in incoming capture to be corrected
         * \return false if the the process discards the correspondence with origin's feature
         */
        virtual bool correctFeatureDrift(const FeatureBase* _last_feature, FeatureBase* _incoming_feature);

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame();

        /** \brief Detect new Features
         * \param _capture_ptr Capture for feature detection. Defaults to incoming_ptr_.
         * \param _new_features_list The list of detected Features. Defaults to member new_features_list_.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function sets the member new_features_list_, the list of newly detected features,
         * to be used for landmark initialization.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_features);

        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _feature_other_ptr FeatureBase pointer to the feature constrained.
         *
         * Implement this method in derived classes.
         * This function only creates the constraint, it doesn't add it to any feature.
         *
         * TODO: Make a general ConstraintFactory, and put it in WolfProblem.
         * This factory only needs to know the two derived pointers to decide on the actual Constraint created.
         */
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, FeatureBase* _feature_other_ptr);

};

inline ProcessorTrackerFeatureDummy::ProcessorTrackerFeatureDummy() :
        ProcessorTrackerFeature(PRC_TRACKER_DUMMY),
        n_feature_(0)
{
    //
}

inline ProcessorTrackerFeatureDummy::~ProcessorTrackerFeatureDummy()
{
    //
}

inline bool ProcessorTrackerFeatureDummy::correctFeatureDrift(const FeatureBase* _last_feature,
                                                              FeatureBase* _incoming_feature)
{
    return true;
}

inline ConstraintBase* ProcessorTrackerFeatureDummy::createConstraint(FeatureBase* _feature_ptr,
                                                                      FeatureBase* _feature_other_ptr)
{
    std::cout << "creating constraint: last feature " << _feature_ptr->getMeasurement()
              << " with origin feature " << _feature_other_ptr->getMeasurement() << std::endl;
    return new ConstraintEpipolar(_feature_ptr, _feature_other_ptr);
}

} // namespace wolf

#endif /* PROCESSOR_TRACKER_FEATURE_DUMMY_H_ */
