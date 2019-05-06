/**
 * \file sensor_odom_3D_yaml.cpp
 *
 *  Created on: Oct 25, 2016
 *      \author: jsola
 */

// wolf yaml
#include "base/yaml/yaml_conversion.h"

// wolf
#include "base/sensor/sensor_odom_3D.h"
#include "base/common/factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{

namespace
{
static IntrinsicsBasePtr createIntrinsicsOdom3D(const std::string & _filename_dot_yaml)
{
    YAML::Node config = YAML::LoadFile(_filename_dot_yaml);

    if (config["sensor type"].as<std::string>() == "ODOM 3D")
    {
        YAML::Node variances = config["motion variances"];

        IntrinsicsOdom3DPtr params = std::make_shared<IntrinsicsOdom3D>();

        params->k_disp_to_disp   = variances["disp_to_disp"] .as<Scalar>();
        params->k_disp_to_rot    = variances["disp_to_rot"]  .as<Scalar>();
        params->k_rot_to_rot     = variances["rot_to_rot"]   .as<Scalar>();
        params->min_disp_var     = variances["min_disp_var"] .as<Scalar>();
        params->min_rot_var      = variances["min_rot_var"]  .as<Scalar>();

        return params;
    }

    std::cout << "Bad configuration file. No sensor type found." << std::endl;
    return nullptr;
}

// Register in the SensorFactory
const bool WOLF_UNUSED registered_odom_3D_intr = IntrinsicsFactory::get().registerCreator("ODOM 3D", createIntrinsicsOdom3D);

} // namespace [unnamed]

} // namespace wolf

