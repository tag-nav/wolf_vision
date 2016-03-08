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

//class CaptureImage
class CaptureImage : public CaptureBase
{
    protected:
        cv::Mat image_;
        cv::Mat descriptors_;
        std::vector<cv::KeyPoint> keypoints_;

    public:
        CaptureImage(const TimeStamp& _ts, SensorCamera* _camera_ptr, cv::Mat _data_cv, int _img_width, int _img_height);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~CaptureImage();

        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now) const;

        //virtual void printSelf(unsigned int _ntabs = 0, std::ostream & _ost = std::cout) const;

        //cv::Mat data_cv_; //this should be protected

        virtual cv::Mat getImage();

        virtual void setDescriptors(cv::Mat _descriptors);

        virtual void setKeypoints(std::vector<cv::KeyPoint> _keypoints);

        virtual cv::Mat getDescriptors();

        virtual std::vector<cv::KeyPoint> getKeypoints();
};

#endif // CAPTURE_IMAGE_H
