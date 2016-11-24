#ifndef PROCESSOR_IMAGE_LANDMARK_H
#define PROCESSOR_IMAGE_LANDMARK_H

// Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"
#include "feature_point_image.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "active_search.h"
#include "processor_tracker_landmark.h"
#include "landmark_AHP.h"
#include "processor_image_params.h"
#include "constraint_AHP.h"

// OpenCV includes
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>


// General includes
#include <cmath>
#include <complex>      // std::complex, std::norm


namespace wolf {

class ProcessorImageLandmark : public ProcessorTrackerLandmark
{
    public:
        typedef std::shared_ptr<ProcessorImageLandmark> Ptr;
    protected:
        std::shared_ptr<cv::DescriptorMatcher> matcher_ptr_;
        std::shared_ptr<cv::Feature2D> detector_descriptor_ptr_;
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

        unsigned int landmarks_tracked_ = 0;

        /* pinhole params */
//        Eigen::Vector4s k_parameters_;
        Eigen::Vector2s distortion_;
        Eigen::Vector2s correction_;

        /* transformations */
        Eigen::Vector3s world2cam_translation_;
        Eigen::Vector4s world2cam_orientation_;

        Eigen::Vector3s cam2world_translation_;
        Eigen::Vector4s cam2world_orientation_;

        // Lists to store values to debug
        std::list<cv::Rect> tracker_roi_;
        std::list<cv::Rect> detector_roi_;
        std::list<cv::Point> tracker_target_;
        FeatureBaseList feat_lmk_found_;

        unsigned int n_feature_;
        unsigned int landmark_idx_non_visible_;

        std::list<float> list_response_;

    public:
        ProcessorImageLandmark(const ProcessorParamsImage& _params);
        virtual ~ProcessorImageLandmark();

        virtual void setup(SensorCamera::Ptr _camera_ptr);

    protected:

        /**
         * \brief Does cast of the images and renews the active grid.
         */
        void preProcess();

        void advance()
        {
            ProcessorTrackerLandmark::advance();
            image_last_ = image_incoming_;
        }

        void reset()
        {
            ProcessorTrackerLandmark::reset();
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
         * \param _landmark_list_in input list of landmarks to be found in incoming
         * \param _feature_list_out returned list of incoming features corresponding to a landmark of _landmark_list_in
         * \param _feature_landmark_correspondences returned map of landmark correspondences: _feature_landmark_correspondences[_feature_out_ptr] = landmark_in_ptr
         */
        virtual unsigned int findLandmarks(const LandmarkBaseList& _landmark_list_in, FeatureBaseList& _feature_list_out,
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
         *
         * This is intended to create Features that are not among the Features already known in the Map.
         *
         * This function sets new_features_last_, the list of newly detected features.
         *
         * \return The number of detected Features.
         */
        virtual unsigned int detectNewFeatures(const unsigned int& _max_new_features);

        /** \brief Create one landmark
         *
         * Implement in derived classes to build the type of landmark you need for this tracker.
         */
        virtual LandmarkBasePtr createLandmark(FeatureBasePtr _feature_ptr);

    public:
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr);



        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         */
        virtual ConstraintBasePtr createConstraint(FeatureBasePtr _feature_ptr, LandmarkBasePtr _landmark_ptr);





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

    private:
        /**
         * \brief Trims the roi of a matrix which exceeds the boundaries of the image
         * \param _roi input/output roi to be trimmed if necessary
         */
        void trimRoi(cv::Rect& _roi);

        /**
         * \brief Augments the designed roi so that the detector and descriptor analize the whole region of interest
         * \param _roi input/output roi to be inflated the necessary amount
         */
        void inflateRoi(cv::Rect& _roi);

        /**
         * \brief Adapts a certain roi to maximize its performance and assign it to the image. It's composed by inflateRoi and trimRoi.
         * \param _image_roi output image to be applied the adapted roi
         * \param _image input image (incoming or last) in which the roi will be applied to obtain \b _image_roi
         * \param _roi input roi to be adapted
         */
        void adaptRoi(cv::Mat& _image_roi, cv::Mat _image, cv::Rect& _roi);

        /**
         * \brief Does the match between a target descriptor and (potentially) multiple candidate descriptors of a Feature.
         * \param _target_descriptor descriptor of the target
         * \param _candidate_descriptors descriptors of the candidates
         * \param _cv_matches output variable in which the best result will be stored (in the position [0])
         * \return normalized score of similarity (1 - exact match; 0 - complete mismatch)
         */
        Scalar match(const cv::Mat _target_descriptor, const cv::Mat _candidate_descriptors, std::vector<cv::DMatch>& _cv_matches);

        void LandmarkInCurrentCamera(CaptureBasePtr _capture, std::shared_ptr<LandmarkAHP> _landmark, Eigen::Vector4s& _point3D_hmg);

        // These only to debug, will disappear one day soon
    public:
        void drawLandmarks(cv::Mat _image);
        void drawFeaturesFromLandmarks(cv::Mat _image);
        void drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color);
        void drawRoi(cv::Mat _image, CaptureImagePtr _capture, cv::Scalar _color);

};


} // namespace wolf


#endif // PROCESSOR_IMAGE_LANDMARK_H
