#include "vision/capture/capture_image.h"

namespace wolf {

CaptureImage::CaptureImage(const TimeStamp& _ts, SensorCameraPtr _camera_ptr, cv::Mat _data_cv) :
    CaptureBase("IMAGE", _ts, _camera_ptr),
    frame_(_data_cv),
    grid_features_(nullptr),
    keypoints_(KeyPointVector()),
    descriptors_(cv::Mat()),
    matches_from_precedent_(DMatchVector()),
    matches_normalized_scores_(std::vector<double>()),
    map_index_to_next_(std::map<int, int>()),
    global_descriptor_(cv::Mat())
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
}

void CaptureImage::setDescriptors(const cv::Mat& _descriptors)
{
    frame_.setDescriptors(_descriptors);
}

void CaptureImage::setKeypoints(const std::vector<cv::KeyPoint> &_keypoints)
{
    frame_.setKeyPoints(_keypoints);
}

cv::Mat& CaptureImage::getDescriptors()
{
    return frame_.getDescriptors();
}

std::vector<cv::KeyPoint>& CaptureImage::getKeypoints()
{
    return frame_.getKeyPoints();
}

void CaptureImage::setGlobalDescriptor(const cv::Mat& _global_descriptor)
{
    global_descriptor_ = _global_descriptor;
}

cv::Mat& CaptureImage::getGlobalDescriptor()
{
    return global_descriptor_;
}

} // namespace wolf
