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
#ifndef CAPTURE_IMAGE_H
#define CAPTURE_IMAGE_H

//Wolf includes
#include <core/capture/capture_base.h>
#include "vision/feature/feature_point_image.h"
#include "vision/sensor/sensor_camera.h"

// Vision Utils includes
#include <vision_utils/vision_utils.h>
#include <vision_utils/common_class/frame.h>

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
    protected:
        vision_utils::Frame frame_;

    public:
        vision_utils::FeatureIdxGridPtr grid_features_;
        KeyPointVector                  keypoints_;
        cv::Mat                         descriptors_;
        DMatchVector                    matches_from_precedent_;
        std::vector<double>             matches_normalized_scores_;
        std::map<int, int>              map_index_to_next_;
        cv::Mat                         global_descriptor_;

    public:
        CaptureImage(const TimeStamp& _ts, SensorCameraPtr _camera_ptr, const cv::Mat& _data_cv);
        ~CaptureImage() override;

        const cv::Mat& getImage() const;
        void setDescriptors(const cv::Mat &_descriptors);
        void setKeypoints(const std::vector<cv::KeyPoint>& _keypoints);
        cv::Mat& getDescriptors();
        const std::vector<cv::KeyPoint>& getKeypoints() const;
        std::vector<cv::KeyPoint>& getKeypoints();
        void setGlobalDescriptor(const cv::Mat &_global_descriptor);
        cv::Mat& getGlobalDescriptor();
};


inline const cv::Mat& CaptureImage::getImage() const
{
    return frame_.getImage();
}

inline void CaptureImage::setDescriptors(const cv::Mat& _descriptors)
{
    frame_.setDescriptors(_descriptors);
}

inline void CaptureImage::setKeypoints(const std::vector<cv::KeyPoint> &_keypoints)
{
    frame_.setKeyPoints(_keypoints);
}

inline cv::Mat& CaptureImage::getDescriptors()
{
    return frame_.getDescriptors();
}

inline std::vector<cv::KeyPoint>& CaptureImage::getKeypoints()
{
    return frame_.getKeyPoints();
}

inline void CaptureImage::setGlobalDescriptor(const cv::Mat& _global_descriptor)
{
    global_descriptor_ = _global_descriptor;
}

inline cv::Mat& CaptureImage::getGlobalDescriptor()
{
    return global_descriptor_;
}

} // namespace wolf

#endif // CAPTURE_IMAGE_H
