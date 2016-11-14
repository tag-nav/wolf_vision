#ifndef CAPTURE_IMAGE_H
#define CAPTURE_IMAGE_H

//Wolf includes
#include "capture_base.h"
#include "feature_point_image.h"
#include "sensor_camera.h"

// opencv includes
#include <opencv2/core/core.hpp>

//std includes
//

namespace wolf {

//forward declaration to typedef class pointers
class CaptureImage;
typedef std::shared_ptr<CaptureImage> CaptureImagePtr;
typedef std::shared_ptr<const CaptureImage> CaptureImageConst;
typedef std::weak_ptr<CaptureImage> CaptureImageWPtr;    
    
/**
 * \brief class CaptureImage
 *
 * This class stores a cv::Mat image, with keypoints and descriptors defined in the OpenCV format.
 * This encapsulation allows this Capture to be used in OpenCV with ease.
 */
class CaptureImage : public CaptureBase
{
    protected:
        cv::Mat image_;
        cv::Mat descriptors_;
        std::vector<cv::KeyPoint> keypoints_;

    public:
        CaptureImage(const TimeStamp& _ts, SensorCamera::Ptr _camera_ptr, cv::Mat _data_cv);
        virtual ~CaptureImage();

        virtual const cv::Mat& getImage() const;
        virtual void setDescriptors(const cv::Mat &_descriptors);
        virtual void setKeypoints(const std::vector<cv::KeyPoint>& _keypoints);
        virtual cv::Mat& getDescriptors();
        virtual std::vector<cv::KeyPoint>& getKeypoints();
};

} // namespace wolf

#endif // CAPTURE_IMAGE_H
