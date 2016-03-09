#ifndef PROCESSOR_BRISK_H
#define PROCESSOR_BRISK_H

// Wolf includes
#include "processor_tracker.h"
#include "sensor_camera.h"
#include "capture_image.h"

// OpenCV includes
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class ProcessorBrisk : public ProcessorTracker
{
protected:
    SensorCamera* sensor_cam_ptr_; //specific pointer to sensor camera object
    CaptureImage* capture_img_ptr_; //specific pointer to capture image object
    cv::BRISK brisk_;               //brisk object

    /** \brief Initialize one landmark
     *
     * Implement in derived classes to build the type of landmark you need for this tracker.
     */
    virtual LandmarkBase* createLandmark(FeatureBase* _feature_ptr);

    /** \brief Create a new constraint
     *
     * Implement in derived classes to build the type of constraint appropriate for the pair feature-landmark used by this tracker.
     */
    virtual ConstraintBase* createConstraint(FeatureBase* _feature_ptr, LandmarkBase* _lmk_ptr);

public:
    ProcessorBrisk(int _threshold = 30, int _octaves = 0, float _pattern_scales = 1.0f);
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
    virtual unsigned int processKnownFeatures(CaptureBase* _incoming_ptr);


    /** \brief Detect new Features
     *
     * This is intended to create Features that are not among the Features already known in the Map.
     * \param _capture_ptr Capture for feature detection
     *
     * This function sets new_features_list_, the list of newly detected features, to be used for landmark initialization.
     *
     * \return The number of detected Features.
     */
    virtual unsigned int detectNewFeatures(CaptureBase* _capture_ptr);


    /** \brief Vote for KeyFrame generation
     *
     * If a KeyFrame criterion is validated, this function returns true,
     * meaning that it wants to create a KeyFrame at the \b last Capture.
     *
     * WARNING! This function only votes! It does not create KeyFrames!
     */
    virtual bool voteForKeyFrame();


    virtual void drawFeatures(cv::Mat _image, std::vector<cv::KeyPoint> _kp, cv::Rect _roi);

    virtual unsigned int briskImplementation(CaptureBase* _capture_ptr, cv::Mat _image, cv::Rect _roi);
};

#endif // PROCESSOR_BRISK_H
