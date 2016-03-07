#include "capture_image.h"
#include <opencv2/core/core.hpp>

CaptureImage::CaptureImage(const TimeStamp& _ts, SensorCamera* _camera_ptr, cv::Mat _data_cv, int _img_width, int _img_height) :
    CaptureBase(_ts, _camera_ptr), image_(_data_cv)
{
    assert((_img_width == _camera_ptr->img_width_ && _img_height == _camera_ptr->img_height_) && "Image and camera sizes don't match");
}

CaptureImage::~CaptureImage()
{
    //
}

Eigen::VectorXs CaptureImage::computeFramePose(const TimeStamp& _now) const
{

    return Eigen::VectorXs::Zero(7);
}

cv::Mat CaptureImage::getImage()
{
    return image_;
}

void CaptureImage::setDescriptors(cv::Mat _descriptors)
{
    descriptors_ = _descriptors;
}

void CaptureImage::setKeypoints(std::vector<cv::KeyPoint> _keypoints)
{
    keypoints_ = _keypoints;
}

cv::Mat CaptureImage::getDescriptors()
{
    return descriptors_;
}

std::vector<cv::KeyPoint> CaptureImage::getKeypoints()
{
    return keypoints_;
}
