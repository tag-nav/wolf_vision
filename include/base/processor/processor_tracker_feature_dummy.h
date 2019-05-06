/**
 * \file processor_tracker_feature_dummy.h
 *
 *  Created on: Apr 11, 2016
 *      \author: jvallve
 */

#ifndef PROCESSOR_TRACKER_FEATURE_DUMMY_H_
#define PROCESSOR_TRACKER_FEATURE_DUMMY_H_

#include "base/common/wolf.h"
#include "base/processor/processor_tracker_feature.h"
#include "base/factor/factor_epipolar.h"

namespace wolf
{
    
WOLF_PTR_TYPEDEFS(ProcessorTrackerFeatureDummy);
    
//Class
class ProcessorTrackerFeatureDummy : public ProcessorTrackerFeature
{

    public:
        ProcessorTrackerFeatureDummy(ProcessorParamsTrackerFeaturePtr _params_tracker_feature);
        virtual ~ProcessorTrackerFeatureDummy();
        virtual void configure(SensorBasePtr _sensor) { };

    protected:

        static unsigned int count_;
        unsigned int n_feature_;

        /** \brief Track provided features from \b last to \b incoming
         * \param _features_last_in input list of features in \b last to track
         * \param _features_incoming_out returned list of features found in \b incoming
         * \param _feature_correspondences returned map of correspondences: _feature_correspondences[feature_out_ptr] = feature_in_ptr
         */
        virtual unsigned int trackFeatures(const FeatureBasePtrList& _features_last_in, FeatureBasePtrList& _features_incoming_out,
                                           FeatureMatchMap& _feature_correspondences);

        /** \brief Correct the drift in incoming feature by re-comparing against the corresponding feature in origin.
         * \param _last_feature input feature in last capture tracked
         * \param _incoming_feature input/output feature in incoming capture to be corrected
         * \return false if the the process discards the correspondence with origin's feature
         */
        virtual bool correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature);

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame();

        /** \brief Detect new Features
         * \param _max_features maximum number of features detected (-1: unlimited. 0: none)
         * \param _features_last_out The list of detected Features.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function is called in ProcessorTrackerFeature::processNew() to set the member new_features_last_,
         * the list of newly detected features of the capture last_ptr_.
         */
        virtual unsigned int detectNewFeatures(const int& _max_features, FeatureBasePtrList& _features_last_out);

        virtual FactorBasePtr createFactor(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr);

};

inline ProcessorTrackerFeatureDummy::ProcessorTrackerFeatureDummy(ProcessorParamsTrackerFeaturePtr _params_tracker_feature) :
        ProcessorTrackerFeature("TRACKER FEATURE DUMMY", _params_tracker_feature),
        n_feature_(0)
{
    //
}

inline ProcessorTrackerFeatureDummy::~ProcessorTrackerFeatureDummy()
{
    //
}

inline bool ProcessorTrackerFeatureDummy::correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature,
                                                              FeatureBasePtr _incoming_feature)
{
    return true;
}

} // namespace wolf

#endif /* PROCESSOR_TRACKER_FEATURE_DUMMY_H_ */
