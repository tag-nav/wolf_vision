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
#ifndef LANDMARK_HP_H
#define LANDMARK_HP_H

//Wolf includes
#include "core/landmark/landmark_base.h"

// yaml
#include <yaml-cpp/yaml.h>

// OpenCV includes
#include <opencv2/core.hpp>
namespace wolf {
    
WOLF_PTR_TYPEDEFS(LandmarkHp);

/* Landmark - Homogeneous Point*/
class LandmarkHp : public LandmarkBase
{
    protected:
        cv::Mat cv_descriptor_;


    public:
        LandmarkHp(Eigen::Vector4d _position_homogeneous, cv::Mat _2d_descriptor);

        ~LandmarkHp() override;

        const cv::Mat& getCvDescriptor() const;
        void setCvDescriptor(const cv::Mat& _descriptor);

        Eigen::Vector3d point() const;

        YAML::Node saveToYaml() const override;

        /** \brief Creator for Factory<LandmarkBase, YAML::Node>
         * Caution: This creator does not set the landmark's sensor.
         * These need to be set afterwards.
         */
        static LandmarkBasePtr create(const YAML::Node& _node);
};


inline const cv::Mat& LandmarkHp::getCvDescriptor() const
{
    return cv_descriptor_;
}


inline void LandmarkHp::setCvDescriptor(const cv::Mat& _descriptor)
{
    cv_descriptor_ = _descriptor;
}

} // namespace wolf

#endif // LANDMARK_AHP_H
