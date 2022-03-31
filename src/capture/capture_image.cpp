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

CaptureImage::CaptureImage(const TimeStamp& _ts, SensorCameraPtr _camera_ptr, const cv::Mat& _img) :
    CaptureBase("CaptureImage", _ts, _camera_ptr),
    img_(_img),
    keypoints_(std::vector<cv::KeyPoint>()),
    descriptors_(cv::Mat()),
    matches_from_precedent_(std::vector<cv::DMatch>()),
    matches_normalized_scores_(std::vector<double>()),
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
    return img_;
}

void CaptureImage::setImage(const cv::Mat& _img)
{
    // Is assignment enough? Use clone or copyTo instead?
    img_ = _img;
}


} // namespace wolf
