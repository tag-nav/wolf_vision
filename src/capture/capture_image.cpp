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


size_t WKeyPoint::id_count_ = 0;

WKeyPoint::WKeyPoint():
    id_(id_count_++)
{
}

WKeyPoint::WKeyPoint(const cv::KeyPoint& _cv_kp):
    id_(id_count_++),
    cv_kp_(_cv_kp)
{
}


CaptureImage::CaptureImage(const TimeStamp& _ts, SensorCameraPtr _camera_ptr, const cv::Mat& _img) :
    CaptureBase("CaptureImage", _ts, _camera_ptr),
    img_(_img),
    mwkps_(KeyPointsMap()),
    descriptors_(cv::Mat()),
    tracks_origin_(TracksMap()),
    tracks_prev_(TracksMap())
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


void CaptureImage::addKeyPoint(const WKeyPoint& _wkp)
{
    mwkps_.insert(std::pair<size_t, WKeyPoint>(_wkp.getId(), _wkp));
}

void CaptureImage::addKeyPoint(const cv::KeyPoint& _cv_kp)
{
    WKeyPoint wkp(_cv_kp); 
    addKeyPoint(wkp);
}

void CaptureImage::addKeyPoints(const std::vector<WKeyPoint>& _vec_wkp)
{
    for (WKeyPoint wkp: _vec_wkp){
        addKeyPoint(wkp);
    }
}

void CaptureImage::addKeyPoints(const std::vector<cv::KeyPoint>& _vec_cv_kp)
{
    for (auto cv_kp: _vec_cv_kp){
        WKeyPoint wkp(cv_kp); 
        addKeyPoint(cv_kp);
    }
}

void CaptureImage::removeKeyPoint(size_t _id)
{
    mwkps_.erase(_id);
}

void CaptureImage::removeKeyPoint(const WKeyPoint& _wkp)
{
    mwkps_.erase(_wkp.getId());
}

} // namespace wolf
