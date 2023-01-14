//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022,2023 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
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
#include "vision/landmark/landmark_point_3d.h"

namespace wolf {

LandmarkPoint3d::LandmarkPoint3d(Eigen::Vector3d _position, cv::Mat _2d_descriptor) :
    LandmarkBase("LandmarkPoint3d", std::make_shared<StateBlock>(_position, false)),
    descriptor_(_2d_descriptor)
{
    //LandmarkPoint3d* landmark_ptr = (LandmarkPoint3d*)_p_ptr;
//    position_ =
//    descriptor_ = _2d_descriptor;
}

LandmarkPoint3d::~LandmarkPoint3d()
{
    //
}

}
