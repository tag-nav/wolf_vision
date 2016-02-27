#ifndef PROCESSOR_IMAGE_POINT_BRISK_H
#define PROCESSOR_IMAGE_POINT_BRISK_H

// Wolf includes
#include "sensor_camera.h"
#include "capture_image.h"

// OpenCV includes
#include "opencv2/features2d/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

class ProcessorImagePointBrisk : public ProcessorBase
{
protected:
    SensorCamera* sensor_cam_ptr_; //specific pointer to sensor camera object
    CaptureImage* capture_img_ptr_; //specific pointer to capture image object
    cv::BRISK brisk_; //brisk object

public:
    ProcessorImagePointBrisk(int _threshold = 30, int _octaves = 0, float _pattern_scales = 1.0f);
    virtual ~ProcessorImagePointBrisk();

    virtual void extractFeatures(CaptureBase *_capture_ptr);
    virtual void establishConstraints(CaptureBase *_capture_ptr);

};
#endif // PROCESSOR_IMAGE_POINT_BRISK_H
