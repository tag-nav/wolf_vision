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
/**
 * \file gtest_factor_epipolar.cpp
 *
 *  Created on: March 31, 2022
 *      \author: mfourmy
 */

#ifndef CAPTURE_IMAGE_H
#define CAPTURE_IMAGE_H

// OpenCV includes
#include <opencv2/core.hpp>


//Wolf includes
#include <core/capture/capture_base.h>
#include "vision/sensor/sensor_camera.h"

namespace wolf {

// Set ClassPtr, ClassConstPtr and ClassWPtr typedefs;
WOLF_PTR_TYPEDEFS(CaptureImage);
    
/**
 * \brief class CaptureImage
 *
 * This class stores a cv::Mat image, with keypoints and descriptors defined in the OpenCV format.
 * This encapsulation allows this Capture to be used in OpenCV with ease.
 */
class CaptureImage : public CaptureBase
{
    private:
        cv::Mat img_;

    public:
        std::vector<cv::KeyPoint>       keypoints_;
        cv::Mat                         descriptors_;
        std::vector<cv::DMatch>         matches_from_precedent_;
        std::vector<double>             matches_normalized_scores_;
        std::map<int, int>              map_index_to_next_;
        cv::Mat                         global_descriptor_;

    public:
        CaptureImage(const TimeStamp& _ts, SensorCameraPtr _camera_ptr, const cv::Mat& _data_cv);
        ~CaptureImage() override;

        const cv::Mat& getImage() const;
        void setImage(const cv::Mat& _img);

        const std::vector<cv::KeyPoint>& getKeypoints() const;
        void setKeypoints(const std::vector<cv::KeyPoint>& _keypoints);

        const cv::Mat& getDescriptors() const;
        void setDescriptors(const cv::Mat &_descriptors);

};

} // namespace wolf

#endif // CAPTURE_IMAGE_H
