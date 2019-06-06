/*
 * processor_bundle_adjustment.h
 *
 *  Created on: May 3, 2019
 *      Author: ovendrell
 */

#ifndef INCLUDE_BASE_PROCESSOR_PROCESSOR_BUNDLE_ADJUSTMENT_H_
#define INCLUDE_BASE_PROCESSOR_PROCESSOR_BUNDLE_ADJUSTMENT_H_

//wolf includes
#include "core/processor/processor_tracker_feature.h"
#include "vision/capture/capture_image.h"
#include "vision/landmark/landmark_HP.h"
#include "vision/math/pinhole_tools.h"
#include "vision/sensor/sensor_camera.h"

//vision utils includes
#include "vision_utils/vision_utils.h"
#include "vision_utils/detectors.h"
#include "vision_utils/descriptors.h"
#include "vision_utils/matchers.h"
#include "../factor/factor_pixelHP.h"

namespace wolf{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsBundleAdjustment);

struct ProcessorParamsBundleAdjustment : public ProcessorParamsTrackerFeature
{
        std::string yaml_file_params_vision_utils;

        bool delete_ambiguities;

        Scalar pixel_noise_std; ///< std noise of the pixel
        int min_track_length_for_factor; ///< Minimum track length of a matched feature to create a factor

};

WOLF_PTR_TYPEDEFS(ProcessorBundleAdjustment);

class ProcessorBundleAdjustment : public ProcessorTrackerFeature
{
    protected:
        vision_utils::DetectorBasePtr det_ptr_;
        vision_utils::DescriptorBasePtr des_ptr_;
        vision_utils::MatcherBasePtr mat_ptr_;

        ProcessorParamsBundleAdjustmentPtr params_bundle_adjustment_;  // Configuration parameters

        CaptureImagePtr capture_image_last_;
        CaptureImagePtr capture_image_incoming_;

        Matrix2s        pixel_cov_;


        //TODO: correct to add this?
        std::map<size_t, LandmarkBasePtr> lmk_track_map_; //LandmarkTrackMap;


    public:
        /** \brief Class constructor
        */
        ProcessorBundleAdjustment(ProcessorParamsBundleAdjustmentPtr _params_bundle_adjustment);
        /** \brief Class destructor
        */
        //virtual ~ProcessorBundleAdjustment();

    public:

        virtual void configure(SensorBasePtr _sensor) override;

        /** Pre-process incoming Capture
         *
         * This is called by process() just after assigning incoming_ptr_ to a valid Capture.
         *
         * Overload this function to prepare stuff on derived classes.
         *
         * Typical uses of prePrecess() are:
         *   - casting base types to derived types
         *   - initializing counters, flags, or any derived variables
         *   - initializing algorithms needed for processing the derived data
         */
        virtual void preProcess() override;

        /** Post-process
         *
         * This is called by process() after finishing the processing algorithm.
         *
         * Overload this function to post-process stuff on derived classes.
         *
         * Typical uses of postPrecess() are:
         *   - resetting and/or clearing variables and/or algorithms at the end of processing
         *   - drawing / printing / logging the results of the processing
         */
        virtual void postProcess() override;

        /** \brief Track provided features from \b last to \b incoming
         * \param _features_last_in input list of features in \b last to track
         * \param _features_incoming_out returned list of features found in \b incoming
         * \param _feature_correspondences returned map of correspondences: _feature_correspondences[feature_out_ptr] = feature_in_ptr
         */
        virtual unsigned int trackFeatures(const FeatureBasePtrList& _features_last_in, FeatureBasePtrList& _features_incoming_out, FeatureMatchMap& _feature_correspondences) override;

        /** \brief Correct the drift in incoming feature by re-comparing against the corresponding feature in origin.
         * \param _origin_feature input feature in origin capture tracked
         * \param _incoming_feature input/output feature in incoming capture to be corrected
         * \return false if the the process discards the correspondence with origin's feature
         */
        virtual bool correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature) override;

        /** \brief Vote for KeyFrame generation
         *
         * If a KeyFrame criterion is validated, this function returns true,
         * meaning that it wants to create a KeyFrame at the \b last Capture.
         *
         * WARNING! This function only votes! It does not create KeyFrames!
         */
        virtual bool voteForKeyFrame() override;

        bool isInlier(const cv::KeyPoint& _kp_incoming, const cv::KeyPoint& _kp_last);

        bool is_tracked(int& kp_idx_);
        /** \brief Detect new Features
         * \param _max_features maximum number of features detected
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * The function sets _features_last_out, the list of newly detected features in Capture last.
         */
        virtual unsigned int detectNewFeatures(const int& _max_new_features, FeatureBasePtrList& _features_last_out) override;

        /** \brief Create a new factor and link it to the wolf tree
         * \param _feature_ptr pointer to the parent Feature
         * \param _feature_other_ptr pointer to the other feature constrained.
         *
         * Implement this method in derived classes.
         *
         * This function creates a factor of the appropriate type for the derived processor.
         */
        virtual FactorBasePtr createFactor(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr) override;

        virtual LandmarkBasePtr createLandmark(FeatureBasePtr _feature_ptr);

        /** \brief Establish factors between features in Captures \b last and \b origin
         */
        virtual void establishFactors() override;

        void setParams(const ProcessorParamsBundleAdjustmentPtr _params);

    public:
        /// @brief Factory method
        static ProcessorBasePtr create(const std::string& _unique_name,
                                       const ProcessorParamsBasePtr _params,
                                       const SensorBasePtr _sensor_ptr);

    private:

        cv::Mat image_debug_;

    public:

        /**
         * \brief Return Image for debug purposes
         */
        cv::Mat getImageDebug();
};

inline cv::Mat ProcessorBundleAdjustment::getImageDebug()
{
    return image_debug_;
}

} //namespace wolf

#endif /* INCLUDE_BASE_PROCESSOR_PROCESSOR_BUNDLE_ADJUSTMENT_H_ */
