#include "capture_image.h"
#include <opencv2/core/core.hpp>

namespace wolf {

CaptureImage::CaptureImage(const TimeStamp& _ts, SensorCamera* _camera_ptr, cv::Mat _data_cv, int _img_width, int _img_height) :
    CaptureBase(_ts, _camera_ptr), image_(_data_cv)
{
    setType("IMAGE");
    //assert((_img_width == _camera_ptr->img_width_ && _img_height == _camera_ptr->img_height_) && "Image and camera sizes don't match");
}

CaptureImage::~CaptureImage()
{
    //
}

const cv::Mat & CaptureImage::getImage() const
{
    return image_;
}

void CaptureImage::setDescriptors(const cv::Mat& _descriptors)
{
    descriptors_ = _descriptors;
}

void CaptureImage::setKeypoints(const std::vector<cv::KeyPoint> &_keypoints)
{
    keypoints_ = _keypoints;
}

cv::Mat CaptureImage::getDescriptors() const
{
    return descriptors_;
}

std::vector<cv::KeyPoint> CaptureImage::getKeypoints() const
{
    return keypoints_;
}



// TODO: This needs to go out some day
Eigen::VectorXs CaptureImage::computeFramePose(const TimeStamp& _now) const
{
    return Eigen::VectorXs::Zero(7);
}

} // namespace wolf
