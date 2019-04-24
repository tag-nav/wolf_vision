#ifndef PROCESSOR_TRACKER_LANDMARK_IMAGE_H
#define PROCESSOR_TRACKER_LANDMARK_IMAGE_H

// Wolf includes

#include "vision/landmark/landmark_AHP.h"
#include "base/landmark/landmark_match.h"
#include "vision/processor/processor_params_image.h"
#include "base/processor/processor_tracker_landmark.h"
#include "base/common/wolf.h"

#include <algorithms/activesearch/alg_activesearch.h>
#include <descriptors/descriptor_base.h>
#include <detectors/detector_base.h>
#include <matchers/matcher_base.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/mat.inl.hpp>
#include <opencv2/core/types.hpp>

#include <list>
#include <memory>
#include <string>
#include <vector>

namespace wolf {

WOLF_PTR_TYPEDEFS(ProcessorTrackerLandmarkImage);

//Class
class ProcessorTrackerLandmarkImage : public ProcessorTrackerLandmark
{
    protected:

        vision_utils::DetectorBasePtr det_ptr_;
        vision_utils::DescriptorBasePtr des_ptr_;
        vision_utils::MatcherBasePtr mat_ptr_;
        vision_utils::AlgorithmACTIVESEARCHPtr active_search_ptr_;  // Active Search

        int cell_width_; ///< Active search cell width
        int cell_height_; ///< Active search cell height
        vision_utils::AlgorithmParamsACTIVESEARCHPtr params_tracker_landmark_image_activesearch_ptr_; ///< Active search parameters

    protected:
        ProcessorParamsTrackerLandmarkImagePtr params_tracker_landmark_image_;           // Struct with parameters of the processors

        cv::Mat image_last_, image_incoming_;   // Images of the "last" and "incoming" Captures

        struct
        {
                unsigned int width_; ///< width of the image
                unsigned int height_; ///< height of the image
        }image_;

        unsigned int landmarks_tracked_ = 0;

        /* pinhole params */
        Eigen::Vector2s distortion_;
        Eigen::Vector2s correction_;

        /* transformations */
        Eigen::Vector3s world2cam_translation_;
        Eigen::Vector4s world2cam_orientation_;

        Eigen::Vector3s cam2world_translation_;
        Eigen::Vector4s cam2world_orientation_;

        unsigned int n_feature_;
        unsigned int landmark_idx_non_visible_;

        std::list<float> list_response_;

    public:

        // Lists to store values to debug
        std::list<cv::Rect> tracker_roi_;
        std::list<cv::Rect> detector_roi_;
        std::list<cv::Point> tracker_target_;
        FeatureBasePtrList feat_lmk_found_;

        ProcessorTrackerLandmarkImage(ProcessorParamsTrackerLandmarkImagePtr _params_tracker_landmark_image);
        virtual ~ProcessorTrackerLandmarkImage();

        virtual void configure(SensorBasePtr _sensor) ;

    protected:

        /**
         * \brief Does cast of the images and renews the active grid.
         */
        void preProcess();

        void advanceDerived()
        {
            ProcessorTrackerLandmark::advanceDerived();
            image_last_ = image_incoming_;
        }

        void resetDerived()
        {
            ProcessorTrackerLandmark::resetDerived();
            image_last_ = image_incoming_;
        }

        /**
         * \brief Does the drawing of the features.
         *
         * Used for debugging
         */
        void postProcess();

        //Pure virtual
        /** \brief Find provided landmarks in the incoming capture
         * \param _landmarks_in input list of landmarks to be found in incoming
         * \param _features_incoming_out returned list of incoming features corresponding to a landmark of _landmarks_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         */
        virtual unsigned int findLandmarks(const LandmarkBasePtrList& _landmarks_in, FeatureBasePtrList& _features_incoming_out,
                                           LandmarkMatchMap& _feature_landmark_correspondences);

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
         * The function is called in ProcessorTrackerLandmark::processNew() to set the member new_features_last_,
         * the list of newly detected features of the capture last_ptr_.
         */
        virtual unsigned int detectNewFeatures(const int& _max_new_features, FeatureBasePtrList& _features_incoming_out);

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBasePtr createLandmark(FeatureBasePtr _feature_ptr);

    public:
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr);

        /** \brief Create a new factor
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         */
        virtual FactorBasePtr createFactor(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr);

        //Other functions
    private:

        /**
         * \brief Detects keypoints its descriptors in a specific roi of the image
         * \param _image input image in which the algorithm will search
         * \param _roi input roi used to define the area of search within the image
         * \param _new_keypoints output keypoints obtained in the function
         * \param new_descriptors output descriptors obtained in the function
         * \return the number of detected features
         */
        unsigned int detect(const cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,cv::Mat& new_descriptors);

        /**
         * \brief Does the match between a target descriptor and (potentially) multiple candidate descriptors of a Feature.
         * \param _target_descriptor descriptor of the target
         * \param _candidate_descriptors descriptors of the candidates
         * \param _cv_matches output variable in which the best result will be stored (in the position [0])
         * \return normalized score of similarity (1 - exact match; 0 - complete mismatch)
         */
        Scalar match(const cv::Mat _target_descriptor, const cv::Mat _candidate_descriptors, std::vector<cv::DMatch>& _cv_matches);

        void landmarkInCurrentCamera(const Eigen::VectorXs& _frame_state,
                                     const LandmarkAHPPtr   _landmark,
                                     Eigen::Vector4s&       _point3D_hmg);

        // These only to debug, will disappear one day soon
    public:
        void drawLandmarks(cv::Mat _image);
        void drawFeaturesFromLandmarks(cv::Mat _image);
        void drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color);
        void drawTrackerRoi(cv::Mat _image, cv::Scalar _color);

};

} // namespace wolf

#endif // PROCESSOR_TRACKER_LANDMARK_IMAGE_H
