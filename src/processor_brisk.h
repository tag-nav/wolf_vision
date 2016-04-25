#ifndef PROCESSOR_BRISK_H
#define PROCESSOR_BRISK_H

// Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"
#include "feature_point_image.h"
#include "state_block.h"
#include "active_search.h"
#include "processor_tracker_feature.h"
#include "constraint_epipolar.h"

// OpenCV includes
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

// General includes
#include <math.h>
#include <complex>      // std::complex, std::norm



namespace wolf {

struct ProcessorImageParameters
{
        struct Image
        {
                unsigned int width;
                unsigned int height;
        }image;
        struct Detector
        {
                unsigned int threshold; ///< on the keypoint strength to declare it key-point
                unsigned int threshold_new_features; ///< on the keypoint strength to declare it key-point
                unsigned int octaves; ///< Multi-scale evaluation. 0: no multi-scale
                unsigned int nominal_pattern_radius; ///< Radius of the detector pattern before scaling
                unsigned int pattern_radius; ///< radius of the pattern used to detect a key-point at pattern_scale = 1.0 and octaves = 0
        }detector;
        struct Descriptor
        {
                unsigned int nominal_pattern_radius; ///< Radius of the descriptor pattern before scaling
                float pattern_scale; ///< Scale of the base pattern wrt the nominal one
                unsigned int pattern_radius; ///< radius of the pattern used to describe a key-point at pattern_scale = 1.0 and octaves = 0
                unsigned int size_bits; ///< length of the descriptor vector in bits
        }descriptor;
        struct Matcher
        {
                Scalar min_normalized_score; ///< 0: perfect match; 1 or -1: awful match; out of [-1,1]: error
                int similarity_norm; ///< Norm used to measure the distance between two descriptors
                unsigned int roi_width; ///< Width of the roi used in the tracking
                unsigned int roi_height; ///< Height of the roi used in the tracking
        }matcher;
        struct Adtive_search
        {
                unsigned int grid_width; ///< cells per horizontal dimension of image
                unsigned int grid_height; ///< cells per vertical dimension of image
                unsigned int separation; ///<
        }active_search;
        struct Algorithm
        {
                unsigned int max_new_features; ///< Max nbr. of features to detect in one frame
                unsigned int min_features_for_keyframe; ///< minimum nbr. of features to vote for keyframe
        }algorithm;
};

class ProcessorBrisk : public ProcessorTrackerFeature
{

//    protected:
//        cv::FeatureDetector* detector_ptr_;
//        cv::DescriptorExtractor* descriptor_ptr_;
//        cv::DescriptorMatcher* matcher_ptr_;
    protected:
        cv::BRISK detector_;                    // Brisk detector
        cv::BRISK descriptor_;                  // Brisk descriptor
        cv::BFMatcher matcher_;                 // Brute force matcher
        ActiveSearchGrid act_search_grid_;      // Active Search
        cv::Mat image_last_, image_incoming_;   // Images from the "last" and "incoming" Captures
        ProcessorImageParameters params_;       // Struct with parameters of the processors

        // Lists to store values to debug
        std::list<cv::Rect> tracker_roi_;
        std::list<cv::Rect> tracker_roi_inflated_;
        std::list<cv::Rect> detector_roi_;
        std::list<cv::Point> tracker_target_;
        std::list<cv::Point> tracker_candidates_;

    public:
        ProcessorBrisk(ProcessorImageParameters _params);

//        ProcessorBrisk(cv::FeatureDetector* _det_ptr, cv::DescriptorExtractor* _desc_ext_ptr, cv::DescriptorMatcher* _match_ptr, ProcessorImageParameters_params) :
//            detector_ptr_(_det_ptr)
//        {}

//        ProcessorBrisk(std::string _detector, std::string _descriptor, std::string matcher, std::string distance, ProcessorImageParameters _params) :
//            detector_ptr_(nullptr), params_(_params)
//        {
//            switch _detector{
//                case "BRISK":
//                    detector_ptr_ = new cv::BRISK();
//                    break;
//                default:
//                    throw runtime_error
//            }
//            }

        virtual ~ProcessorBrisk();

//        virtual ~ProcessorBrisk(){
//            delete detector_ptr_; delete descriptor_ptr_; //etc
//        };

    protected:
        void preProcess();
        void postProcess();

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
         *
         * This is intended to create Features that are not among the Features already known in the Map.
         *
         * This function sets new_features_last_, the list of newly detected features, to be used for landmark initialization.
         *
         * \return The number of detected Features.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_new_features);

        /** \brief Create a new constraint
         *
         * Implement in derived classes to build the type of constraint appropriate for the pair feature-landmark used by this tracker.
         */
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, FeatureBase* _feature_other_ptr);

    private:
        virtual unsigned int detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,
                                         cv::Mat& new_descriptors);

    private:
        virtual void trimRoi(cv::Rect& _roi);
        virtual void inflateRoi(cv::Rect& _roi);
        virtual void adaptRoi(cv::Mat& _image_roi, cv::Mat _image, cv::Rect& _roi);








        // These only to debug, will disappear one day soon
    public:
        virtual void drawFeatures(CaptureBase* const _last_ptr);

        virtual void drawTrackingFeatures(cv::Mat _image, std::list<cv::Point> _target_list, std::list<cv::Point> _candidates_list);

        virtual void drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color);

        virtual void resetVisualizationFlag(FeatureBaseList& _feature_list_last,
                                            FeatureBaseList& _feature_list_incoming);



};

inline bool ProcessorBrisk::voteForKeyFrame()
{
//    std::cout << "voteForKeyFrame?: "
//            << (((CaptureImage*)((incoming_ptr_)))->getFeatureListPtr()->size() < params_.algorithm.min_features_for_keyframe) << std::endl;
    return (incoming_ptr_->getFeatureListPtr()->size() < params_.algorithm.min_features_for_keyframe);
}

inline ConstraintBase* ProcessorBrisk::createConstraint(FeatureBase* _feature_ptr, FeatureBase* _feature_other_ptr)
{
    ConstraintEpipolar* const_epipolar_ptr = new ConstraintEpipolar(_feature_ptr, _feature_other_ptr);
    return const_epipolar_ptr; // TODO Crear constraint
}

} // namespace wolf


#endif // PROCESSOR_BRISK_H
