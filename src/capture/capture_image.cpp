//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
#include "vision/capture/capture_image.h"

namespace wolf {

CaptureImage::CaptureImage(const TimeStamp& _ts, SensorCameraPtr _camera_ptr, const cv::Mat& _data_cv) :
    CaptureBase("CaptureImage", _ts, _camera_ptr),
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
