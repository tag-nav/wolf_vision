//--------LICENSE_START--------
//
// Copyright (C) 2020,2021 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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
#include "vision/landmark/landmark_ahp.h"

#include "core/state_block/state_homogeneous_3d.h"
#include "core/common/factory.h"
#include "core/yaml/yaml_conversion.h"

namespace wolf {

/* Landmark - Anchored Homogeneous Point*/
LandmarkAhp::LandmarkAhp(Eigen::Vector4d _position_homogeneous,
                         FrameBasePtr _anchor_frame,
                         SensorBasePtr _anchor_sensor,
                         cv::Mat _2d_descriptor) :
    LandmarkBase("AHP", std::make_shared<StateHomogeneous3d>(_position_homogeneous)),
    cv_descriptor_(_2d_descriptor.clone()),
    anchor_frame_(_anchor_frame),
    anchor_sensor_(_anchor_sensor)
{
}

LandmarkAhp::~LandmarkAhp()
{
    //
}

YAML::Node LandmarkAhp::saveToYaml() const
{
    // First base things
    YAML::Node node = LandmarkBase::saveToYaml();

    // Then add specific things
    std::vector<int> v;
    LandmarkAhp::cv_descriptor_.copyTo(v);
    node["descriptor"] = v;
    return node;
}

Eigen::Vector3d LandmarkAhp::getPointInAnchorSensor() const
{
    Eigen::Map<const Eigen::Vector4d> hmg_point(getP()->getState().data());
    return hmg_point.head<3>()/hmg_point(3);
}

Eigen::Vector3d LandmarkAhp::point() const
{
    using namespace Eigen;
    Transform<double,3,Isometry> T_w_r
        = Translation<double,3>(getAnchorFrame()->getP()->getState())
        * Quaterniond(getAnchorFrame()->getO()->getState().data());
    Transform<double,3,Isometry> T_r_c
        = Translation<double,3>(getAnchorSensor()->getP()->getState())
        * Quaterniond(getAnchorSensor()->getO()->getState().data());
    Vector4d point_hmg_c = getP()->getState();
    Vector4d point_hmg = T_w_r * T_r_c * point_hmg_c;
    return point_hmg.head<3>()/point_hmg(3);
}

LandmarkBasePtr LandmarkAhp::create(const YAML::Node& _node)
{
    unsigned int        id          = _node["id"]           .as< unsigned int     >();
    Eigen::VectorXd     pos_homog   = _node["position"]     .as< Eigen::VectorXd  >();
    std::vector<int>    v           = _node["descriptor"]   .as< std::vector<int> >();
    cv::Mat desc(v);

    LandmarkBasePtr lmk = std::make_shared<LandmarkAhp>(pos_homog, nullptr, nullptr, desc);
    lmk->setId(id);
    return lmk;
}

// Register landmark creator
namespace
{
const bool WOLF_UNUSED registered_lmk_ahp = FactoryLandmark::registerCreator("LandmarkAhp", LandmarkAhp::create);
}

} // namespace wolf
