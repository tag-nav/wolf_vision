#ifndef PROCESSOR_IMAGE_FEATURE_H
#define PROCESSOR_IMAGE_FEATURE_H

// Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"
#include "feature_point_image.h"
#include "state_block.h"
#include "active_search.h"
#include "processor_tracker_feature.h"
#include "constraint_epipolar.h"
#include "processor_image_params.h"


// OpenCV includes
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>


// General includes
#include <cmath>
#include <complex>      // std::complex, std::norm



namespace wolf {

class ProcessorImageFeature : public ProcessorTrackerFeature
{
    public:
        typedef std::shared_ptr<ProcessorImageFeature> Ptr;

    protected:
        cv::DescriptorMatcher* matcher_ptr_;
        cv::Feature2D* detector_descriptor_ptr_;
    protected:
        ProcessorParamsImage params_;           // Struct with parameters of the processors
        ActiveSearchGrid active_search_grid_;   // Active Search
        cv::Mat image_last_, image_incoming_;   // Images of the "last" and "incoming" Captures
        struct
        {
                unsigned int pattern_radius_; ///< radius of the pattern used to detect a key-point at pattern_scale = 1.0 and octaves = 0
                unsigned int size_bits_; ///< length of the descriptor vector in bits
        }detector_descriptor_params_;
        struct
        {
                unsigned int width_; ///< width of the image
                unsigned int height_; ///< height of the image
        }image_;

        // Lists to store values to debug
        std::list<cv::Rect> tracker_roi_;
        std::list<cv::Rect> tracker_roi_inflated_;
        std::list<cv::Rect> detector_roi_;
        std::list<cv::Point> tracker_target_;
        unsigned int complete_target_size_ = 1;
        unsigned int target_size_ = 1;

    public:
        ProcessorImageFeature(ProcessorParamsImage _params);
        virtual ~ProcessorImageFeature();

        virtual void setup(SensorCamera::Ptr _camera_ptr);

    protected:

        /**
         * \brief Does cast of the images and renews the active grid.
         */
        void preProcess();

        /**
         * \brief Does the drawing of the features.
         *
         * Used for debugging
         */
        void postProcess();

        void advance()
        {
            ProcessorTrackerFeature::advance();
            image_last_ = image_incoming_;
        }

        void reset()
        {
            ProcessorTrackerFeature::reset();
            image_last_ = image_incoming_;
        }

        virtual unsigned int trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out,
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
         *
         * This is intended to create Features that are not among the Features already known in the Map.
         *
         * This function sets new_features_last_, the list of newly detected features.
         *
         * \return The number of detected Features.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_new_features);

        /** \brief Create a new constraint
         *
         * Creates a constraint from feature to feature
         */
        virtual ConstraintBasePtr createConstraint(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr);

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
                                         cv::Mat& new_descriptors);

    private:
        /**
         * \brief Trims the roi of a matrix which exceeds the boundaries of the image
         * \param _roi input/output roi to be trimmed if necessary
         */
        virtual void trimRoi(cv::Rect& _roi);

        /**
         * \brief Augments the designed roi so that the detector and descriptor analize the whole region of interest
         * \param _roi input/output roi to be inflated the necessary amount
         */
        virtual void inflateRoi(cv::Rect& _roi);

        /**
         * \brief Adapts a certain roi to maximize its performance and assign it to the image. It's composed by inflateRoi and trimRoi.
         * \param _image_roi output image to be applied the adapted roi
         * \param _image input image (incoming or last) in which the roi will be applied to obtain \b _image_roi
         * \param _roi input roi to be adapted
         */
        virtual void adaptRoi(cv::Mat& _image_roi, cv::Mat _image, cv::Rect& _roi);

        /**
         * \brief Does the match between a target descriptor and (potentially) multiple candidate descriptors of a Feature.
         * \param _target_descriptor descriptor of the target
         * \param _candidate_descriptors descriptors of the candidates
         * \param _cv_matches output variable in which the best result will be stored (in the position [0])
         * \return normalized score of similarity (1 - exact match; 0 - complete mismatch)
         */
        virtual Scalar match(cv::Mat _target_descriptor, cv::Mat _candidate_descriptors, std::vector<cv::DMatch>& _cv_matches);


        // These only to debug, will disappear one day
    public:
        virtual void drawFeatures(cv::Mat _image);
        virtual void drawTarget(cv::Mat _image, std::list<cv::Point> _target_list);
        virtual void drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color);
        virtual void resetVisualizationFlag(FeatureBaseList& _feature_list_last);

    public:
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params);


};

inline bool ProcessorImageFeature::voteForKeyFrame()
{
    return (incoming_ptr_->getFeatureList().size() < params_.algorithm.min_features_for_keyframe);
}

inline ConstraintBasePtr ProcessorImageFeature::createConstraint(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr)
{
    std::shared_ptr<ConstraintEpipolar> const_epipolar_ptr = std::make_shared<ConstraintEpipolar>(_feature_ptr, _feature_other_ptr);
    const_epipolar_ptr->setFeatureOtherPtr(_feature_other_ptr);
    return const_epipolar_ptr;
}

} // namespace wolf


#endif // PROCESSOR_IMAGE_FEATURE_H
