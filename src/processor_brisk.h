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

struct ImageTrackerParameters
{
        struct Image
        {
                unsigned int width;
                unsigned int height;
        }image;
        struct Detector
        {
                unsigned int threshold; ///< on the keypoint strength to declare it key-point
                unsigned int octaves; ///< Multi-scale evaluation. 0: no multi-scale
                float pattern_scales; ///< Scale of the base pattern wrt the nominal one
                unsigned int pattern_radius; ///< radius of the pattern used to detect a key-point at scale = 1.0
        }detector;
        struct Descriptor
        {
                unsigned int size; ///< size of the descriptor vector (length of the vector)
                unsigned int octaves; ///< Multi-scale evaluation. 0: no multi-scale
                float pattern_scales; ///< Scale of the base pattern wrt the nominal one
                unsigned int pattern_radius; ///< radius of the pattern used to compute the descriptor at the nominal scale
        }descriptor;
        struct Matcher
        {
                Scalar max_similarity_distance; ///< 0: perfect match; 1 or -1: awful match; out of [-1,1]: error
        }matcher;
        struct Adtive_search
        {
                unsigned int grid_width; ///< cells per horizontal dimension of image
                unsigned int grid_height; ///< cells per vertical dimension of image
                unsigned int separation;
        }active_search;
        struct Algorithm
        {
                unsigned int max_new_features; ///< Max nbr. of features to detect in one frame
                unsigned int min_features_th; ///< minimum nbr. of features to vote for keyframe
        }algorithm;
};

class ProcessorBrisk : public ProcessorTrackerFeature
{
    protected:
        cv::BRISK detector_;    //brisk detector
        cv::BRISK descriptor_;    //brisk descriptor
        cv::BFMatcher matcher_; // Brute force matcher
        ActiveSearchGrid act_search_grid_;
        unsigned int min_features_th_;
//        bool known_or_new_features_ = false;
        cv::Mat image_last_, image_incoming_;
        std::list<cv::Rect> tracker_roi_;
        std::list<FeaturePointImage*> tracker_features_;
        unsigned int img_width_;
        unsigned int img_height_;
    public:
        ProcessorBrisk(unsigned int _image_rows, unsigned int _image_cols,
                       unsigned int _grid_width = 8, unsigned int _grid_height = 8, unsigned int _separation = 5,
                       unsigned int _max_new_features = 20, unsigned int _min_features_th = 10,
                       int _threshold = 30, int _octaves = 0, float _pattern_scales = 1.0f, unsigned int _adjust = 10);
        virtual ~ProcessorBrisk();

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
         * \param _capture_ptr Capture for feature detection
         *
         * This function sets new_features_list_, the list of newly detected features, to be used for landmark initialization.
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

    public:
        virtual void drawFeatures(CaptureBase* const _last_ptr);

        virtual void drawTrackingFeatures(cv::Mat _image, std::list<FeaturePointImage*> _features_list);

        virtual void drawRoiLastFrame(cv::Mat _image, std::list<cv::Rect> _roi_list);

        virtual void resetVisualizationFlag(FeatureBaseList& _feature_list_last,
                                            FeatureBaseList& _feature_list_incoming);

        virtual void assureRoi(cv::Rect& _roi);

        virtual void inflateRoi(cv::Mat& _image_roi, cv::Mat _image, cv::Rect& _roi);


};

inline bool ProcessorBrisk::voteForKeyFrame()
{
    std::cout << "voteForKeyFrame?: "
            << (((CaptureImage*)((incoming_ptr_)))->getFeatureListPtr()->size() < min_features_th_) << std::endl;
    return (incoming_ptr_->getFeatureListPtr()->size() < min_features_th_);
}

inline ConstraintBase* ProcessorBrisk::createConstraint(FeatureBase* _feature_ptr, FeatureBase* _feature_other_ptr)
{
    ConstraintEpipolar* const_epipolar_ptr = new ConstraintEpipolar(_feature_ptr, _feature_other_ptr);
    return const_epipolar_ptr; // TODO Crear constraint
}

} // namespace wolf


#endif // PROCESSOR_BRISK_H
