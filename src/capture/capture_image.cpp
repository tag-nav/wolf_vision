#include "capture_image.h"

namespace wolf {

CaptureImage::CaptureImage(const TimeStamp& _ts, SensorCameraPtr _camera_ptr, cv::Mat _data_cv) :
    CaptureBase("IMAGE", _ts, _camera_ptr),
    frame_(_data_cv),
    grid_features_(nullptr),
    keypoints_(KeyPointVector()),
    descriptors_(cv::Mat()),
    matches_from_precedent_(DMatchVector()),
    matches_normalized_scores_(std::vector<Scalar>()),
    map_index_to_next_(std::map<int, int>())
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


} // namespace wolf
