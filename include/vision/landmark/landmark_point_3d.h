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
#ifndef LANDMARK_POINT_3d_H
#define LANDMARK_POINT_3d_H


// OpenCV includes
#include <opencv2/core.hpp>

//Wolf includes
#include "core/landmark/landmark_base.h"


namespace wolf {

WOLF_PTR_TYPEDEFS(LandmarkPoint3d);
    
//class    
class LandmarkPoint3d : public LandmarkBase
{
    protected:
        cv::Mat descriptor_;
        Eigen::Vector3d position_;
    public:
        LandmarkPoint3d(Eigen::Vector3d _position, cv::Mat _2d_descriptor);

        ~LandmarkPoint3d() override;

        const Eigen::Vector3d point() const;

        const cv::Mat& getDescriptor() const;
        void setDescriptor(const cv::Mat& _descriptor);
};

inline const Eigen::Vector3d LandmarkPoint3d::point() const
{
    return getP()->getState();
}

inline const cv::Mat& LandmarkPoint3d::getDescriptor() const
{
    return descriptor_;
}

inline void LandmarkPoint3d::setDescriptor(const cv::Mat& _descriptor)
{
    descriptor_ = _descriptor;
}

} // namespace wolf

#endif // LANDMARK_POINT_3d_H
