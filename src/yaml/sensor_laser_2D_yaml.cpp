/**
 * \file sensor_laser_2D_yaml.cpp
 *
 *  Created on: May 13, 2016
 *      \author: jsola
 */


// wolf yaml
#include "yaml_conversion.h"

// wolf
#include "../intrinsics_factory.h"
#include "../sensor_laser_2D.h"

// yaml library
#include <yaml-cpp/yaml.h>

namespace wolf
{
namespace {
// intrinsics creator
IntrinsicsBase* createIntrinsicsLaser2D(const std::string& _filename_dot_yaml)
{
    // TODO: Parse YAML <-- maybe we want this out of this file?
    IntrinsicsLaser2D* params; // dummy
    params->type = "LASER 2D"; // fill this one just for the fun of it
    return params;
}


// register into factory
const bool registered_laser_params = IntrinsicsFactory::get().registerCreator("LASER 2D", createIntrinsicsLaser2D);

} // namespace [void]
} // namespace wolf
