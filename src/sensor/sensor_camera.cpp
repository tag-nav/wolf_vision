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

#include "vision/sensor/sensor_camera.h"
#include "vision/math/pinhole_tools.h"

#include "core/state_block/state_block.h"
#include "core/state_block/state_quaternion.h"

namespace wolf
{

SensorCamera::SensorCamera(const Eigen::VectorXd& _extrinsics, const ParamsSensorCamera& _intrinsics) :
                SensorBase("SensorCamera", 
                           std::make_shared<StateBlock>(_extrinsics.head(3), true), 
                           std::make_shared<StateQuaternion>(_extrinsics.tail(4), true), 
                           std::make_shared<StateBlock>(_intrinsics.pinhole_model_raw, true), 
                           1),
                img_width_(_intrinsics.width), //
                img_height_(_intrinsics.height), //
                distortion_(_intrinsics.distortion), //
                correction_(distortion_.size()==0 ? 0 : distortion_.size() + 1), // make correction vector slightly larger in size than the distortion vector
                pinhole_model_raw_(_intrinsics.pinhole_model_raw), //
                pinhole_model_rectified_(_intrinsics.pinhole_model_rectified), //
                using_raw_(true)
{
    assert(_extrinsics.size() == 7 && "Wrong intrinsics vector size. Should be 7 for 3d");
    useRawImages();
    pinhole::computeCorrectionModel(getIntrinsic()->getState(), distortion_, correction_);
}

SensorCamera::SensorCamera(const Eigen::VectorXd& _extrinsics, ParamsSensorCameraPtr _intrinsics_ptr) :
        SensorCamera(_extrinsics, *_intrinsics_ptr)
{
    //
}

SensorCamera::~SensorCamera()
{
    //
}

Eigen::Matrix3d SensorCamera::setIntrinsicMatrix(Eigen::Vector4d _pinhole_model)
{
    Eigen::Matrix3d K;
    K(0, 0) = _pinhole_model(2);
    K(0, 1) = 0;
    K(0, 2) = _pinhole_model(0);
    K(1, 0) = 0;
    K(1, 1) = _pinhole_model(3);
    K(1, 2) = _pinhole_model(1);
    K.row(2) << 0, 0, 1;
    return K;
}
} // namespace wolf

// Register in the FactorySensor
#include "core/sensor/factory_sensor.h"
namespace wolf
{
WOLF_REGISTER_SENSOR(SensorCamera)
WOLF_REGISTER_SENSOR_AUTO(SensorCamera)
} // namespace wolf

