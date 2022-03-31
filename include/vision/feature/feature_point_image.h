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

#ifndef FEATURE_POINT_IMAGE_H
#define FEATURE_POINT_IMAGE_H

// OpenCV includes
#include <opencv2/core.hpp>

//Wolf includes
#include "core/feature/feature_base.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(FeaturePointImage);
    
//class FeaturePointImage
class FeaturePointImage : public FeatureBase
{
    private:
        cv::KeyPoint keypoint_; ///< Warning: every write operation to this member needs to write measurement_. See setKeypoint() as an example.
        std::size_t index_keypoint_;
        cv::Mat descriptor_;

    public:

        /// Constructor from Eigen measured pixel
        FeaturePointImage(const Eigen::Vector2d & _measured_pixel,
                          const std::size_t& _index_keypoint,
                          const cv::Mat& _descriptor,
                          const Eigen::Matrix2d& _meas_covariance) :
                FeatureBase("FeaturePointImage", _measured_pixel, _meas_covariance),
                keypoint_(_measured_pixel(0), _measured_pixel(1), 1), // Size 1 is a dummy value
                index_keypoint_(_index_keypoint),
                descriptor_( _descriptor)
        {
            keypoint_.pt.x = measurement_(0);
            keypoint_.pt.y = measurement_(1);
        }

        /// Constructor from OpenCV measured keypoint
        FeaturePointImage(const cv::KeyPoint& _keypoint,
                          const std::size_t& _index_keypoint,
                          const cv::Mat& _descriptor,
                          const Eigen::Matrix2d& _meas_covariance) :
                FeatureBase("FeaturePointImage", Eigen::Vector2d::Zero(), _meas_covariance),
                keypoint_(_keypoint),
                index_keypoint_(_index_keypoint),
                descriptor_(_descriptor)
        {
            measurement_(0) = double(_keypoint.pt.x);
            measurement_(1) = double(_keypoint.pt.y);
        }

       ~FeaturePointImage() override;

        const cv::KeyPoint& getKeypoint() const;
        void setKeypoint(const cv::KeyPoint& _kp);

        const cv::Mat& getDescriptor() const;
        void setDescriptor(const cv::Mat& _descriptor);

        std::size_t getIndexKeyPoint() const;
        void setIndexKeyPoint(std::size_t _index);
};

inline const cv::KeyPoint& FeaturePointImage::getKeypoint() const
{
    return keypoint_;
}

inline void FeaturePointImage::setKeypoint(const cv::KeyPoint& _kp)
{
    keypoint_ = _kp;
    measurement_(0) = _kp.pt.x;
    measurement_(1) = _kp.pt.y;
}

inline const cv::Mat& FeaturePointImage::getDescriptor() const
{
    return descriptor_;
}

inline void FeaturePointImage::setDescriptor(const cv::Mat& _descriptor)
{
    descriptor_ = _descriptor;
}

std::size_t FeaturePointImage::getIndexKeyPoint() const
{
    return index_keypoint_;
}

void FeaturePointImage::setIndexKeyPoint(std::size_t _index)
{
     index_keypoint_ = _index;
}


} // namespace wolf

#endif // FEATURE_POINT_IMAGE_H
