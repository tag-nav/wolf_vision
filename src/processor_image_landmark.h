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
#include "constraint_epipolar.h"
#include "landmark_AHP.h"

// OpenCV includes
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>


// General includes
#include <cmath>
#include <complex>      // std::complex, std::norm


namespace wolf {

enum DetectorDescriptorType
{
    DD_BRISK,
    DD_ORB
};

struct DetectorDescriptorParamsBase
{
        DetectorDescriptorType type; ///< Type of algorithm. Accepted values in wolf.h
        unsigned int nominal_pattern_radius = 0; ///< Radius of the pattern before scaling //18 for brisk
        //should this be here? doesn't it depend on the descriptor?
};

struct DetectorDescriptorParamsBrisk : public DetectorDescriptorParamsBase
{
        unsigned int threshold=30; ///< on the keypoint strength to declare it key-point
        unsigned int octaves=0; ///< Multi-scale evaluation. 0: no multi-scale
        float pattern_scale=1.0f; ///< Scale of the base pattern wrt the nominal one
};

struct DetectorDescriptorParamsOrb : public DetectorDescriptorParamsBase
{
        unsigned int nfeatures=500; ///< Nbr of features to extract
        float scaleFactor=1.2f; ///< Scale factor between two consecutive scales of the image pyramid
        unsigned int nlevels=1;///< Number of levels in the pyramid. Default: 8
        unsigned int edgeThreshold=4; ///< ? //Default: 31
        unsigned int firstLevel=0;
        unsigned int WTA_K=2;
        unsigned int scoreType=cv::ORB::HARRIS_SCORE; ///< Type of score to rank the detected points
        unsigned int patchSize=31;
};

struct ProcessorImageParameters : public ProcessorParamsBase
{
        struct Image
        {
                unsigned int width; ///< image width (horizontal dimension or nbr of columns)
                unsigned int height; ///< image height (vertical dimension or nbr of rows)
        }image;

        DetectorDescriptorParamsBase* detector_descriptor_params_ptr;

        struct Matcher
        {
                Scalar min_normalized_score; ///< [-1..0]: awful match; 1: perfect match; out of [-1,1]: error
                int similarity_norm; ///< Norm used to measure the distance between two descriptors
                unsigned int roi_width; ///< Width of the roi used in tracking
                unsigned int roi_height; ///< Height of the roi used in tracking
        }matcher;
        struct Adtive_search
        {
                unsigned int grid_width; ///< cells per horizontal dimension of image
                unsigned int grid_height; ///< cells per vertical dimension of image
                unsigned int separation; ///< Distance between the border of the cell and the border of the associated ROI
        }active_search;
        struct Algorithm
        {
                unsigned int max_new_features; ///< Max nbr. of features to detect in one frame
                unsigned int min_features_for_keyframe; ///< minimum nbr. of features to vote for keyframe
        }algorithm;
//        struct Pinhole_params
//        {
//                Eigen::Vector4s k_parameters;
//                Eigen::Vector2s distortion;
//        }pinhole_params;
};

class ProcessorImageLandmark : public ProcessorTrackerLandmark
{
    protected:
        cv::DescriptorMatcher* matcher_ptr_;
        cv::Feature2D* detector_descriptor_ptr_;
    protected:
        ProcessorImageParameters params_;       // Struct with parameters of the processors
        ActiveSearchGrid active_search_grid_;   // Active Search
        cv::Mat image_last_, image_incoming_;   // Images of the "last" and "incoming" Captures
        struct
        {
                unsigned int pattern_radius_; ///< radius of the pattern used to detect a key-point at pattern_scale = 1.0 and octaves = 0
                unsigned int size_bits_; ///< length of the descriptor vector in bits
        }detector_descriptor_params_;

        /* pinhole params */
        Eigen::Vector4s k_parameters_;
        Eigen::Vector2s distortion_;
        Eigen::Vector2s correction_;

        /* transformations */
        Eigen::Vector3s world2cam_translation_;
        Eigen::Vector4s world2cam_orientation_;

        Eigen::Vector3s cam2world_translation_;
        Eigen::Vector4s cam2world_orientation_;

        // Lists to store values to debug
        std::list<cv::Rect> tracker_roi_;
        std::list<cv::Rect> tracker_roi_inflated_;
        std::list<cv::Rect> detector_roi_;
        std::list<cv::Point> tracker_target_;
        std::list<cv::Point> tracker_candidates_;

        unsigned int n_feature_;
        unsigned int landmark_idx_non_visible_;

        unsigned int landmarks_in_image_ = 0;

    public:
        ProcessorImageLandmark(ProcessorImageParameters _params);
        virtual ~ProcessorImageLandmark();

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
            ProcessorTrackerLandmark::advance();
            image_last_ = image_incoming_;
        }

        void reset()
        {
            ProcessorTrackerLandmark::reset();
            image_last_ = image_incoming_;
        }


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
        virtual LandmarkBase* createLandmark(FeatureBase* _feature_ptr);

    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);



        /** \brief Create a new constraint
         * \param _feature_ptr pointer to the Feature to constrain
         * \param _landmark_ptr LandmarkBase pointer to the Landmark constrained.
         *
         *
         *
         * TODO: Make a general ConstraintFactory, and put it in WolfProblem.
         * This factory only needs to know the two derived pointers to decide on the actual Constraint created.
         */
        virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _landmark_ptr);





        //Other functions

        /** \brief Correct the drift in incoming feature by re-comparing against the corresponding feature in origin.
         * \param _last_feature input feature in last capture tracked
         * \param _incoming_feature input/output feature in incoming capture to be corrected
         * \return false if the the process discards the correspondence with origin's feature
         */
//        virtual bool correctFeatureDrift(const FeatureBase* _origin_feature, const FeatureBase* _last_feature, FeatureBase* _incoming_feature);

    private:

        /**
         * \brief Detects keypoints its descriptors in a specific roi of the image
         * \param _image input image in which the algorithm will search
         * \param _roi input roi used to define the area of search within the image
         * \param _new_keypoints output keypoints obtained in the function
         * \param new_descriptors output descriptors obtained in the function
         * \return the number of detected features
         */
        virtual unsigned int detect(cv::Mat _image, cv::Rect& _roi, std::vector<cv::KeyPoint>& _new_keypoints,cv::Mat& new_descriptors);

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

        virtual Scalar match(cv::Mat _target_descriptor, cv::Mat _candidate_descriptors, std::vector<cv::KeyPoint> _candidate_keypoints, std::vector<cv::DMatch>& _cv_matches);

//        virtual void filterFeatureLists(FeatureBaseList _original_list, FeatureBaseList& _filtered_list);

        virtual void referenceWorldToCamera(Eigen::Vector3s& _wc_translation, Eigen::Vector4s& _wc_orientation);

        virtual void referenceCameraToWorld(Eigen::Vector3s& _cw_translation, Eigen::Vector4s& _cw_orientation);

        virtual void rotationMatrix(Eigen::Matrix3s& _rotation_matrix, Eigen::Vector4s _orientation);

        virtual void quaternionProduct(Eigen::Vector4s _p, Eigen::Vector4s _q, Eigen::Vector4s& _quaternion_product);

        virtual void world2CameraFrameTransformation(Eigen::Vector3s _wc_translation, Eigen::Vector4s _wc_orientation, Eigen::Vector3s& _point3D);

        virtual void camera2WorldFrameTransformation(Eigen::Vector3s _cw_translation, Eigen::Vector4s _cw_orientation, Eigen::Vector3s& _point3D);

        virtual void changeOfReference(LandmarkAHP* _landmark, Eigen::Vector3s _wc_translation, Eigen::Vector4s _wc_orientation, Eigen::Vector3s& _point3D);

        // These only to debug, will disappear one day soon
    public:
        virtual void drawFeatures(cv::Mat& _image);

        virtual void drawTrackingFeatures(cv::Mat _image, std::list<cv::Point> _target_list, std::list<cv::Point> _candidates_list);

        virtual void drawRoi(cv::Mat _image, std::list<cv::Rect> _roi_list, cv::Scalar _color);

//        virtual void resetVisualizationFlag(FeatureBaseList& _feature_list_last);



};


} // namespace wolf


#endif // PROCESSOR_IMAGE_LANDMARK_H
