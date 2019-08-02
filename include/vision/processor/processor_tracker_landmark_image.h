#ifndef PROCESSOR_TRACKER_LANDMARK_IMAGE_H
#define PROCESSOR_TRACKER_LANDMARK_IMAGE_H

// Wolf includes

#include "vision/landmark/landmark_AHP.h"
#include "core/landmark/landmark_match.h"
#include "vision/processor/processor_params_image.h"
#include "core/processor/processor_tracker_landmark.h"
#include "core/common/wolf.h"

#include <vision_utils/algorithms/activesearch/alg_activesearch.h>
#include <vision_utils/descriptors/descriptor_base.h>
#include <vision_utils/detectors/detector_base.h>
#include <vision_utils/matchers/matcher_base.h>

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

        virtual void configure(SensorBasePtr _sensor) override;

    protected:

        /**
         * \brief Does cast of the images and renews the active grid.
         */
        void preProcess() override;

        void advanceDerived() override
        {
            ProcessorTrackerLandmark::advanceDerived();
            image_last_ = image_incoming_;
        }

        void resetDerived() override
        {
            ProcessorTrackerLandmark::resetDerived();
            image_last_ = image_incoming_;
        }

        /**
         * \brief Does the drawing of the features.
         *
         * Used for debugging
         */
        void postProcess() override;

        /** \brief Find provided landmarks as features in the provided capture
         * \param _landmarks_in input list of landmarks to be found
         * \param _capture the capture in which the _landmarks_in should be searched
         * \param _features_out returned list of features  found in \b _capture corresponding to a landmark of _landmarks_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         *
         * \return the number of landmarks found
         */
        virtual unsigned int findLandmarks(const LandmarkBasePtrList& _landmarks_in,
                                           const CaptureBasePtr& _capture,
                                           FeatureBasePtrList& _features_out,
                                           LandmarkMatchMap& _feature_landmark_correspondences) override;

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
         * If you detect all the features at once in `preprocess()`, you should either emplace them (`FeatureBase::emplace()`) and remove the not returned features in _features_out (`FeatureBase::remove()`),
         * or create them (`make_shared()`) and link all the returned features in _features_out (`FeatureBase::link(_capture)`).
         *
         * The function is called in ProcessorTrackerLandmark::processNew() to set the member new_features_last_,
         * the list of newly detected features of the capture last_ptr_.
         */
        virtual unsigned int detectNewFeatures(const int& _max_new_features,
                                               const CaptureBasePtr& _capture,
                                               FeatureBasePtrList& _features_out) override;

        /** \brief Emplaces one landmark
         */
        virtual LandmarkBasePtr emplaceLandmark(FeatureBasePtr _feature_ptr) override;

    public:
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params);

        /** \brief Emplaces a new factor
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         */
        virtual FactorBasePtr emplaceFactor(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr) override;

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
