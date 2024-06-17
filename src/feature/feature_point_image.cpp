//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023,2024 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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

#include "vision/feature/feature_point_image.h"

namespace wolf {

/// Constructor from OpenCV measured keypoint
FeaturePointImage::FeaturePointImage(const WKeyPoint& _keypoint,
                                     const Eigen::Matrix2d& _meas_covariance) :
        FeatureBase("FeaturePointImage", Eigen::Vector2d::Zero(), _meas_covariance),
        kp_(_keypoint)
{
    measurement_(0) = double(_keypoint.getCvKeyPoint().pt.x);
    measurement_(1) = double(_keypoint.getCvKeyPoint().pt.y);
}

FeaturePointImage::~FeaturePointImage()
{
    //
}

} // namespace wolf
