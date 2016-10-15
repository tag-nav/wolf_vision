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

/**
 * \brief class CaptureImage
 *
 * This class stores a cv::Mat image, with keypoints and descriptors defined in the OpenCV format.
 * This encapsulation allows this Capture to be used in OpenCV with ease.
 */
class CaptureImage : public CaptureBase
{
        typedef std::shared_ptr<SensorCamera> SensorCameraPtr;
    protected:
        cv::Mat image_;
        cv::Mat descriptors_;
        std::vector<cv::KeyPoint> keypoints_;

    public:
        CaptureImage(const TimeStamp& _ts, SensorCameraPtr _camera_ptr, cv::Mat _data_cv);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         */
        virtual ~CaptureImage();

        virtual const cv::Mat& getImage() const;
        virtual void setDescriptors(const cv::Mat &_descriptors);
        virtual void setKeypoints(const std::vector<cv::KeyPoint>& _keypoints);
        virtual cv::Mat& getDescriptors();
        virtual std::vector<cv::KeyPoint>& getKeypoints();


};

} // namespace wolf

#endif // CAPTURE_IMAGE_H
