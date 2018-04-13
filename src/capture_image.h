#ifndef CAPTURE_IMAGE_H
#define CAPTURE_IMAGE_H

//Wolf includes
#include "capture_base.h"
#include "feature_point_image.h"
#include "sensor_camera.h"

// Vision Utils includes
#include <vision_utils.h>
#include <common_class/frame.h>

namespace wolf {

// Set ClassPtr, ClassConstPtr and ClassWPtr typedefs;
WOLF_PTR_TYPEDEFS(CaptureImage);
    
/**
 * \brief class CaptureImage
 *
 * This class stores a cv::Mat image, with keypoints and descriptors defined in the OpenCV format.
 * This encapsulation allows this Capture to be used in OpenCV with ease.
 */
class CaptureImage : public CaptureBase
{
    protected:
        vision_utils::Frame frame_;

    public:
        CaptureImage(const TimeStamp& _ts, SensorCameraPtr _camera_ptr, cv::Mat _data_cv);
        virtual ~CaptureImage();

        const cv::Mat& getImage() const;
        void setDescriptors(const cv::Mat &_descriptors);
        void setKeypoints(const std::vector<cv::KeyPoint>& _keypoints);
        cv::Mat& getDescriptors();
        std::vector<cv::KeyPoint>& getKeypoints();
};

} // namespace wolf

#endif // CAPTURE_IMAGE_H
