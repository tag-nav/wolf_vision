/**
 * \file sensor_laser_2D_yaml.cpp
 *
 *  Created on: May 13, 2016
 *      \author: jsola
 */

// wolf yaml
#include "base/yaml/yaml_conversion.h"

// wolf
//#include "base/intrinsics_factory.h"
#include "base/common/factory.h"
#include "base/sensor/sensor_laser_2D.h"

// yaml library
#include <yaml-cpp/yaml.h>

namespace wolf
{
namespace {
// intrinsics creator
IntrinsicsBasePtr createIntrinsicsLaser2D(const std::string& _filename_dot_yaml)
{
    // If required: Parse YAML

    IntrinsicsLaser2DPtr params; // dummy
    return params;
}

// register into factory
const bool WOLF_UNUSED registered_laser_params = IntrinsicsFactory::get().registerCreator("LASER 2D", createIntrinsicsLaser2D);

} // namespace [void]
} // namespace wolf
