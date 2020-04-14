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
#include "vision/landmark/landmark_hp.h"
#include "vision/math/pinhole_tools.h"
#include "vision/sensor/sensor_camera.h"
#include "core/math/rotations.h"

//vision utils includes
#include "vision_utils/vision_utils.h"
#include "vision_utils/detectors.h"
#include "vision_utils/descriptors.h"
#include "vision_utils/matchers.h"
#include "../factor/factor_pixel_hp.h"

namespace wolf{

WOLF_STRUCT_PTR_TYPEDEFS(ParamsProcessorBundleAdjustment);

struct ParamsProcessorBundleAdjustment : public ParamsProcessorTrackerFeature
{
	std::string yaml_file_params_vision_utils;

	bool delete_ambiguities;

	int n_cells_h;
	int n_cells_v;
	int min_response_new_feature;

	double pixel_noise_std; ///< std noise of the pixel
	int min_track_length_for_factor; ///< Minimum track length of a matched feature to create a factor

	ParamsProcessorBundleAdjustment() = default;

};

WOLF_PTR_TYPEDEFS(ProcessorBundleAdjustment);

class ProcessorBundleAdjustment : public ProcessorTrackerFeature
{
    protected:
        vision_utils::DetectorBasePtr det_ptr_;
        vision_utils::DescriptorBasePtr des_ptr_;
        vision_utils::MatcherBasePtr mat_ptr_;

        ParamsProcessorBundleAdjustmentPtr params_bundle_adjustment_;  // Configuration parameters

        CaptureImagePtr capture_image_last_;
        CaptureImagePtr capture_image_incoming_;

        SensorCameraPtr camera;

        Matrix2d        pixel_cov_;

        cv::Mat tvec_;
        cv::Mat rvec_;


        //TODO: correct to add this?
        std::map<size_t, LandmarkBasePtr> lmk_track_map_; //LandmarkTrackMap;

	public:
		vision_utils::DetectorBasePtr   get_det_ptr_() {return det_ptr_;};
        vision_utils::DescriptorBasePtr get_des_ptr_() {return des_ptr_;};
        vision_utils::MatcherBasePtr    get_mat_ptr_() {return mat_ptr_;};

    private:
        int frame_count_;


    public:
        /** \brief Class constructor
        */
        ProcessorBundleAdjustment(ParamsProcessorBundleAdjustmentPtr _params_bundle_adjustment);
        /** \brief Class destructor
        */
        virtual ~ProcessorBundleAdjustment()
        {
            //
        }

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

        /** \brief Track provided features in \b _capture
         * \param _features_in input list of features in \b last to track
         * \param _capture the capture in which the _features_in should be searched
         * \param _features_out returned list of features found in \b _capture
         * \param _feature_correspondences returned map of correspondences: _feature_correspondences[feature_out_ptr] = feature_in_ptr
         *
         * \return the number of features tracked
         */
        virtual unsigned int trackFeatures(const FeatureBasePtrList& _features_in,
                                           const CaptureBasePtr& _capture,
                                           FeatureBasePtrList& _features_out,
                                           FeatureMatchMap& _feature_correspondences) override;

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
        virtual bool voteForKeyFrame() const override;

        bool isInlier(const cv::KeyPoint& _kp_incoming, const cv::KeyPoint& _kp_last) const;

        bool is_tracked(int& kp_idx_) const;

        /** \brief Detect new Features
         * \param _max_features maximum number of features detected (-1: unlimited. 0: none)
         * \param _capture The capture in which the new features should be detected.
         * \param _features_out The list of detected Features in _capture.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
         *
         * IMPORTANT: The features in _features_out should be emplaced. Don't use `make_shared`, use `FeatureBase::emplace` instead.
         * Then, they will be already linked to the _capture.
         * If you detect all the features at once in preprocess(), you should either emplace them (`FeatureBase::emplace()`) and remove the not returned features in _features_out (`FeatureBase::remove()`),
         * or create them (`make_shared()`) and link all the returned features in _features_out (`FeatureBase::link(_capture)`).
         *
         * The function is called in ProcessorTrackerFeature::processNew() to set the member new_features_last_,
         * the list of newly detected features of the capture last_ptr_.
         */
        virtual unsigned int detectNewFeatures(const int& _max_new_features,
                                               const CaptureBasePtr& _capture,
                                               FeatureBasePtrList& _features_out) override;

        /** \brief Emplaces a new factor
         * \param _feature_ptr pointer to the parent Feature
         * \param _feature_other_ptr pointer to the other feature constrained.
         *
         * Implement this method in derived classes.
         *
         * This function emplaces a factor of the appropriate type for the derived processor.
         */
        virtual FactorBasePtr emplaceFactor(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr) override;

        virtual LandmarkBasePtr emplaceLandmark(FeatureBasePtr _feature_ptr);

        /** \brief Establish factors between features in Captures \b last and \b origin
         */
        virtual void establishFactors() override;

        void setParams(const ParamsProcessorBundleAdjustmentPtr _params);

    public:
        /// @brief Factory method
        static ProcessorBasePtr create(const std::string& _unique_name,
                                       const ParamsProcessorBasePtr _params);

    private:

        cv::Mat image_debug_;

    public:

        /**
         * \brief Return Image for debug purposes
         */
        cv::Mat getImageDebug() const;

        /**
         * \brief Return list of Features tracked in a Capture
         */
        std::list<FeatureBasePtr> trackedFeatures(const CaptureBasePtr& _capture_ptr) const;
        /**
        * \brief Return list of Landmarks
        */
        std::list<LandmarkBasePtr> currentLandmarks() const;
};

inline cv::Mat ProcessorBundleAdjustment::getImageDebug() const
{
    return image_debug_;
}

inline std::list<FeatureBasePtr> ProcessorBundleAdjustment::trackedFeatures(const CaptureBasePtr& _capture_ptr) const
{
	return track_matrix_.snapshotAsList(_capture_ptr);
}

inline std::list<LandmarkBasePtr> ProcessorBundleAdjustment::currentLandmarks() const
{
	return getProblem()->getMap()->getLandmarkList();
}

} //namespace wolf

#endif /* INCLUDE_BASE_PROCESSOR_PROCESSOR_BUNDLE_ADJUSTMENT_H_ */
