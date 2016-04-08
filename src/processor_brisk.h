#ifndef PROCESSOR_BRISK_H
#define PROCESSOR_BRISK_H

// Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"
#include "feature_point_image.h"
#include "state_block.h"
#include "active_search.h"
#include "processor_tracker_feature.h"

// OpenCV includes
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

// General includes
#include <math.h>
#include <complex>      // std::complex, std::norm


class ProcessorBrisk : public ProcessorTrackerFeature
{
protected:
    cv::BRISK brisk_;               //brisk object
    ActiveSearchGrid act_search_grid_;
    unsigned int min_features_th_;
    bool known_or_new_features_ = false;
    cv::Mat image_last_, image_incoming_;

    /** \brief Initialize one landmark
     *
     * Implement in derived classes to build the type of landmark you need for this tracker.
     */
    virtual LandmarkBase* createLandmark(FeatureBase* _feature_ptr);

    /** \brief Create a new constraint
     *
     * Implement in derived classes to build the type of constraint appropriate for the pair feature-landmark used by this tracker.
     */
    virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, FeatureBase* _feat_or_lmk_ptr);


public:
    ProcessorBrisk(unsigned int _image_rows, unsigned int _image_cols,
                   unsigned int _grid_width = 8, unsigned int _grid_height = 8, unsigned int _min_features_th = 10,
                   int _threshold = 30, int _octaves = 0, float _pattern_scales = 1.0f);
    virtual ~ProcessorBrisk();


    /** \brief Tracker function
     *
     * This is the tracker function to be implemented in derived classes. It operates on the incoming capture.
     *
     * This should do one of the following, depending on the design of the tracker:
     *   - Track Features against other Features in another Capture.
     *   - Track Features against Landmarks in the Map.
     *
     * It should also generate the necessary Features in the incoming Capture, of a type derived from FeatureBase,
     * and the constraints, of a type derived from ConstraintBase.
     *
     * \return The number of successful tracks.
     */
    virtual unsigned int processKnown();


    /** \brief Detect new Features
     *
     * This is intended to create Features that are not among the Features already known in the Map.
     * \param _capture_ptr Capture for feature detection
     *
     * This function sets new_features_list_, the list of newly detected features, to be used for landmark initialization.
     *
     * \return The number of detected Features.
     */
    virtual unsigned int detectNewFeatures();


    /** \brief Vote for KeyFrame generation
     *
     * If a KeyFrame criterion is validated, this function returns true,
     * meaning that it wants to create a KeyFrame at the \b last Capture.
     *
     * WARNING! This function only votes! It does not create KeyFrames!
     */
    virtual bool voteForKeyFrame();


    virtual void drawFeatures(CaptureBase* const _last_ptr);

    virtual void drawTrackingFeatures(cv::Mat _image, Eigen::Vector2i _feature_point, bool _is_candidate);

    virtual void drawRoiLastFrame(cv::Mat _image, cv::Rect _roi);

    virtual void resetVisualizationFlag(FeatureBaseList& _feature_list_last,FeatureBaseList& _feature_list_incoming);

    virtual void addNewFeaturesInCapture(cv::KeyPoint _new_keypoints, cv::Mat _new_descriptors); //std::vector<cv::KeyPoint>

    virtual unsigned int briskDetect(cv::Mat _image, cv::Rect &_roi, std::vector<cv::KeyPoint> &_new_keypoints, cv::Mat & new_descriptors);

    virtual void process(CaptureBase* const _incoming_ptr);

private:

    virtual unsigned int track(const FeatureBaseList& _feature_list_in, FeatureBaseList & _feature_list_out);
};

#endif // PROCESSOR_BRISK_H
