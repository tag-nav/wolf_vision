#ifndef _PROCESSOR_TRACKER_FEATURE_TRIFOCAL_H_
#define _PROCESSOR_TRACKER_FEATURE_TRIFOCAL_H_

//Wolf includes
#include "core/processor/processor_tracker_feature.h"
#include "vision/capture/capture_image.h"
#include "core/utils/params_server.hpp"

// Vision utils
#include <vision_utils/vision_utils.h>
#include <vision_utils/detectors/detector_base.h>
#include <vision_utils/descriptors/descriptor_base.h>
#include <vision_utils/matchers/matcher_base.h>
#include <vision_utils/algorithms/activesearch/alg_activesearch.h>

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsTrackerFeatureTrifocal);

struct ProcessorParamsTrackerFeatureTrifocal : public ProcessorParamsTrackerFeature
{
        std::string yaml_file_params_vision_utils;

        int n_cells_h;
        int n_cells_v;
        int min_response_new_feature;
        Scalar pixel_noise_std; ///< std noise of the pixel
        int min_track_length_for_factor; ///< Minimum track length of a matched feature to create a factor
    ProcessorParamsTrackerFeatureTrifocal() = default;
    ProcessorParamsTrackerFeatureTrifocal(std::string _unique_name, const paramsServer& _server):
        ProcessorParamsTrackerFeature(_unique_name, _server)
    {
        yaml_file_params_vision_utils = _server.getParam<std::string>(_unique_name + "/yaml_file_params_vision_utils");
        n_cells_h = _server.getParam<int>(_unique_name + "/n_cells_h");
        n_cells_v = _server.getParam<int>(_unique_name + "/n_cells_v");
        min_response_new_feature = _server.getParam<int>(_unique_name + "/min_response_new_feature");
        pixel_noise_std = _server.getParam<Scalar>(_unique_name + "/pixel_noise_std");
        min_track_length_for_factor = _server.getParam<int>(_unique_name + "/min_track_length_for_factor");
    }
    std::string print()
    {
        return "\n" + ProcessorParamsTrackerFeature::print()
            + "yaml_file_params_vision_utils: " + yaml_file_params_vision_utils + "\n"
            + "n_cells_h: " + std::to_string(n_cells_h) + "\n"
            + "n_cells_v: " + std::to_string(n_cells_v) + "\n"
            + "min_response_new_feature: " + std::to_string(min_response_new_feature) + "\n"
            + "pixel_noise_std: " + std::to_string(pixel_noise_std) + "\n"
            + "min_track_length_for_factor: " + std::to_string(min_track_length_for_factor) + "\n";
    }
};

WOLF_PTR_TYPEDEFS(ProcessorTrackerFeatureTrifocal);

class ProcessorTrackerFeatureTrifocal : public ProcessorTrackerFeature
{
        // Parameters for vision_utils
    protected:
        vision_utils::DetectorBasePtr   det_ptr_;
        vision_utils::DescriptorBasePtr des_ptr_;
        vision_utils::MatcherBasePtr    mat_ptr_;

    protected:

        ProcessorParamsTrackerFeatureTrifocalPtr params_tracker_feature_trifocal_;      ///< Configuration parameters

        CaptureImagePtr capture_image_last_;
        CaptureImagePtr capture_image_incoming_;
        Matrix2s        pixel_cov_;

    private:
        CaptureBasePtr prev_origin_ptr_;                    ///< Capture previous to origin_ptr_ for the third focus of the trifocal.
        bool initialized_;                                  ///< Flags the situation where three focus are available: prev_origin, origin, and last.

    public:

        /** \brief Class constructor
         */
        ProcessorTrackerFeatureTrifocal( ProcessorParamsTrackerFeatureTrifocalPtr _params_tracker_feature_trifocal );

        /** \brief Class Destructor
         */
        virtual ~ProcessorTrackerFeatureTrifocal();
        virtual void configure(SensorBasePtr _sensor) override;

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
        virtual bool voteForKeyFrame() override;

        /** \brief Detect new Features
         * \param _max_features maximum number of features detected (-1: unlimited. 0: none)
         * \param _capture The capture in which the new features should be detected.
         * \param _features_out The list of detected Features in _capture.
         * \return The number of detected Features.
         *
         * This function detects Features that do not correspond to known Features/Landmarks in the system.
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
         * This function emplaces a factor of the appropriate type for the derived processor.
         */
        virtual FactorBasePtr emplaceFactor(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr) override;

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        /** \brief advance pointers
         *
         */
        virtual void advanceDerived() override;

        /** \brief reset pointers and match lists at KF creation
         *
         */
        virtual void resetDerived() override;

        /** \brief Pre-process: check if all captures (prev-origin, origin, last) are initialized to allow factors creation
         *
         */
        virtual void preProcess() override;

        /** \brief Post-process
         *
         */
        virtual void postProcess() override;

        /** \brief Establish factors between features in Captures \b last and \b origin
         */
        virtual void establishFactors() override;

        CaptureBasePtr getPrevOrigin();

        bool isInlier(const cv::KeyPoint& _kp_incoming, const cv::KeyPoint& _kp_last);

        void setParams(const ProcessorParamsTrackerFeatureTrifocalPtr _params);

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

        /// \brief Get pixel covariance
        const Matrix2s& pixelCov() const;
};

inline CaptureBasePtr ProcessorTrackerFeatureTrifocal::getPrevOrigin()
{
    return prev_origin_ptr_;
}

inline cv::Mat ProcessorTrackerFeatureTrifocal::getImageDebug()
{
    return image_debug_;
}

inline const Eigen::Matrix2s& ProcessorTrackerFeatureTrifocal::pixelCov() const
{
    return pixel_cov_;
}

} // namespace wolf

#endif /* _PROCESSOR_TRACKER_FEATURE_TRIFOCAL_H_ */
