// WOLF - Copyright (C) 2020,2021,2022,2023
// Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu) and
// Joan Vallvé Navarro (jvallve@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF: http://www.iri.upc.edu/wolf
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

#pragma once

// yaml
#include <yaml-cpp/yaml.h>

// OpenCV includes
#include <opencv2/core.hpp>

//Wolf includes
#include "vision/common/vision.h"
#include "core/landmark/landmark_base.h"


namespace wolf {
    
WOLF_PTR_TYPEDEFS(LandmarkAhp);

/* Landmark - Anchored Homogeneous Point*/
class LandmarkAhp : public LandmarkBase
{
    protected:
        cv::Mat cv_descriptor_;
        FrameBasePtr anchor_frame_;
        SensorBasePtr anchor_sensor_;

    public:
        LandmarkAhp(Eigen::Vector4d _position_homogeneous, FrameBasePtr _anchor_frame, SensorBasePtr _anchor_sensor, cv::Mat _2d_descriptor);

        ~LandmarkAhp() override;

        const cv::Mat& getCvDescriptor() const;
        void setCvDescriptor(const cv::Mat& _descriptor);

        FrameBaseConstPtr  getAnchorFrame () const;
        FrameBasePtr  getAnchorFrame ();
        SensorBaseConstPtr getAnchorSensor() const;
        SensorBasePtr getAnchorSensor();

        void setAnchorFrame  (FrameBasePtr  _anchor_frame );
        void setAnchorSensor (SensorBasePtr _anchor_sensor);
        void setAnchor       (FrameBasePtr  _anchor_frame , SensorBasePtr _anchor_sensor);
        Eigen::Vector3d getPointInAnchorSensor() const;
        Eigen::Vector3d point() const;

        YAML::Node toYaml() const override;

        /** \brief Creator for Factory<LandmarkBase, YAML::Node>
         * Caution: This creator does not set the landmark's anchor frame and sensor.
         * These need to be set afterwards.
         */
        static LandmarkBasePtr create(const YAML::Node& _node);
};

inline const cv::Mat& LandmarkAhp::getCvDescriptor() const
{
    return cv_descriptor_;
}

inline void LandmarkAhp::setCvDescriptor(const cv::Mat& _descriptor)
{
    cv_descriptor_ = _descriptor;
}

inline FrameBaseConstPtr LandmarkAhp::getAnchorFrame() const
{
    return anchor_frame_;
}

inline FrameBasePtr LandmarkAhp::getAnchorFrame() 
{
    return anchor_frame_;
}

inline void LandmarkAhp::setAnchorFrame(FrameBasePtr _anchor_frame)
{
    anchor_frame_ = _anchor_frame;
}

inline SensorBaseConstPtr LandmarkAhp::getAnchorSensor() const
{
    return anchor_sensor_;
}

inline SensorBasePtr LandmarkAhp::getAnchorSensor()
{
    return anchor_sensor_;
}

inline void LandmarkAhp::setAnchorSensor(SensorBasePtr _anchor_sensor)
{
    anchor_sensor_ = _anchor_sensor;
}

inline void LandmarkAhp::setAnchor(FrameBasePtr _anchor_frame, SensorBasePtr _anchor_sensor)
{
    anchor_frame_  = _anchor_frame;
    anchor_sensor_ = _anchor_sensor;
}

} // namespace wolf