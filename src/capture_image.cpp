#include "capture_image.h"
#include <opencv2/core/core.hpp>

namespace wolf {

CaptureImage::CaptureImage(const TimeStamp& _ts, SensorCamera* _camera_ptr, cv::Mat _data_cv) :
    CaptureBase(_ts, _camera_ptr), image_(_data_cv)
{
    setType("IMAGE");
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

cv::Mat& CaptureImage::getDescriptors()
{
    return descriptors_;
}

std::vector<cv::KeyPoint>& CaptureImage::getKeypoints()
{
    return keypoints_;
}



// TODO: This needs to go out some day
Eigen::VectorXs CaptureImage::computeFramePose(const TimeStamp& _now) const
{
    return Eigen::VectorXs::Zero(7);
}

} // namespace wolf
