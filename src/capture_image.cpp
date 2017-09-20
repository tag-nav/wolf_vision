#include "capture_image.h"
#include <opencv2/core/core.hpp>

namespace wolf {

CaptureImage::CaptureImage(const TimeStamp& _ts, SensorCameraPtr _camera_ptr, cv::Mat _data_cv) :
    CaptureBase("IMAGE", _ts, _camera_ptr), frame_(_data_cv)
{
    //
}

CaptureImage::~CaptureImage()
{
    //
}

const cv::Mat& CaptureImage::getImage() const
{
    return frame_.getImage();
//    REMOVE
//    return image_;
}

void CaptureImage::setDescriptors(const cv::Mat& _descriptors)
{
    frame_.setDescriptors(_descriptors);
//    REMOVE
//    descriptors_ = _descriptors;
}

void CaptureImage::setKeypoints(const std::vector<cv::KeyPoint> &_keypoints)
{
    frame_.setKeyPoints(_keypoints);
//    REMOVE
//    keypoints_ = _keypoints;
}

cv::Mat& CaptureImage::getDescriptors()
{
    return frame_.getDescriptors();
//    REMOVE
//    return descriptors_;
}

std::vector<cv::KeyPoint>& CaptureImage::getKeypoints()
{
    return frame_.getKeyPoints();
//    REMOVE
//    return keypoints_;
}


} // namespace wolf
