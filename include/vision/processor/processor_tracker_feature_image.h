#ifndef PROCESSOR_TRACKER_FEATURE_IMAGE_H
#define PROCESSOR_TRACKER_FEATURE_IMAGE_H

// Wolf includes
#include "vision/sensor/sensor_camera.h"
#include "vision/capture/capture_image.h"
#include "vision/feature/feature_point_image.h"
#include "vision/processor/processor_params_image.h"
#include <core/state_block/state_block.h>
#include <core/state_block/state_quaternion.h>
#include <core/processor/processor_tracker_feature.h>
#include <core/dummy/factor_feature_dummy.h>
#include "vision/processor/processor_params_image.h"


// vision_utils
#include <vision_utils/detectors/detector_base.h>
#include <vision_utils/descriptors/descriptor_base.h>
#include <vision_utils/matchers/matcher_base.h>
#include <vision_utils/algorithms/activesearch/alg_activesearch.h>

// General includes
#include <cmath>
#include <complex>      // std::complex, std::norm

namespace wolf
{

WOLF_PTR_TYPEDEFS(ProcessorTrackerFeatureImage);

//class
class ProcessorTrackerFeatureImage : public ProcessorTrackerFeature
{
        // vision utils params
    protected:

        vision_utils::DetectorBasePtr det_ptr_;
        vision_utils::DescriptorBasePtr des_ptr_;
        vision_utils::MatcherBasePtr mat_ptr_;
        vision_utils::AlgorithmACTIVESEARCHPtr active_search_ptr_;  // Active Search

        int cell_width_; ///< Active search cell width
        int cell_height_; ///< Active search cell height
        vision_utils::AlgorithmParamsACTIVESEARCHPtr params_tracker_feature_image_activesearch_ptr_; ///< Active search parameters

    protected:

        ProcessorParamsTrackerFeatureImagePtr params_tracker_feature_image_;           ///< Struct with parameters of the processors
        cv::Mat image_last_, image_incoming_;   ///< Images of the "last" and "incoming" Captures

        struct
        {
                unsigned int width_; ///< width of the image
                unsigned int height_; ///< height of the image
        } image_;

    public:

        // Lists to store values to debug
        std::list<cv::Rect> tracker_roi_;
        std::list<cv::Rect> tracker_roi_inflated_;
        std::list<cv::Rect> detector_roi_;
        std::list<cv::Point> tracker_target_;

        ProcessorTrackerFeatureImage(ProcessorParamsTrackerFeatureImagePtr _params_image);
        virtual ~ProcessorTrackerFeatureImage();

        virtual void configure(SensorBasePtr _sensor) override ;

    protected:

        /**
         * \brief Does cast of the images and renews the active grid.
         */
        void preProcess() override;

        /**
         * \brief Does the drawing of the features.
         *
         * Used for debugging
         */
        void postProcess() override;

        void advanceDerived() override
        {
            ProcessorTrackerFeature::advanceDerived();
            image_last_ = image_incoming_;
        }

        void resetDerived() override
        {
            ProcessorTrackerFeature::resetDerived();
            image_last_ = image_incoming_;
        }

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
         * \param _last_feature input feature in last capture tracked
         * \param _incoming_feature input/output feature in incoming capture to be corrected
         * \return false if the the process discards the correspondence with origin's feature
         */
        virtual bool correctFeatureDrift(const FeatureBasePtr _origin_feature,
                                         const FeatureBasePtr _last_feature,
                                         FeatureBasePtr _incoming_feature) override;

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

    private:

        /**
         * \brief Detects keypoints its descriptors in a specific roi of the image
         * \param _image input image in which the algorithm will search
         * \param _roi input roi used to define the area of search within the image
         * \param _new_keypoints output keypoints obtained in the function
         * \param new_descriptors output descriptors obtained in the function
         * \return the number of detected features
         */
        virtual unsigned int detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
                                    cv::Mat& _new_descriptors);

        /**
         * \brief Does the match between a target descriptor and (potentially) multiple candidate descriptors of a Feature.
         * \param _target_descriptor descriptor of the target
         * \param _candidate_descriptors descriptors of the candidates
         * \param _cv_matches output variable in which the best result will be stored (in the position [0])
         * \return normalized score of similarity (1 - exact match; 0 - complete mismatch)
         */
        virtual Scalar match(cv::Mat _target_descriptor, cv::Mat _candidate_descriptors,
                             std::vector<cv::DMatch>& _cv_matches);

        // These only to debug, will disappear one day
    public:
        virtual void drawFeatures(cv::Mat _image);
        virtual void drawTarget(cv::Mat _image, std::list<cv::Point> _target_list);
        virtual void drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color);
        virtual void resetVisualizationFlag(const FeatureBasePtrList& _feature_list_last);

    public:
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params,
                                       const SensorBasePtr sensor_ptr = nullptr);

};

inline bool ProcessorTrackerFeatureImage::voteForKeyFrame()
{
    return (incoming_ptr_->getFeatureList().size() < params_tracker_feature_image_->min_features_for_keyframe);
}

} // namespace wolf

#endif // PROCESSOR_TRACKER_FEATURE_IMAGE_H
