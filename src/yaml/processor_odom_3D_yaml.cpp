/**
 * \file processor_odom_3D_yaml.cpp
 *
 *  Created on: Oct 25, 2016
 *      \author: jsola
 */

// wolf yaml
#include "yaml_conversion.h"

// wolf
#include "../processor_odom_3D.h"
#include "../factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{

namespace
{
static ProcessorParamsBasePtr createProcessorOdom3DParams(const std::string & _filename_dot_yaml)
{
    YAML::Node config = YAML::LoadFile(_filename_dot_yaml);

    if (config["processor type"].as<std::string>() == "ODOM 3D")
    {

        // YAML:: to Eigen::
        using namespace Eigen;
        std::string processor_type = config["processor type"]     .as<std::string>();
        std::string processor_name = config["processor name"]     .as<std::string>();

        YAML::Node kf_vote = config["keyframe vote"];

        ProcessorOdom3DParamsPtr params = std::make_shared<ProcessorOdom3DParams>();

        params->type                = processor_type;
        params->name                = processor_name;
        params->max_time_span       = kf_vote["max time span"]      .as<Scalar>();
        params->max_buff_length     = kf_vote["max buffer length"]  .as<Size  >();
        params->dist_traveled       = kf_vote["dist traveled"]      .as<Scalar>();
        params->angle_turned        = kf_vote["angle turned"]       .as<Scalar>();

        return params;
    }

    std::cout << "Bad configuration file. No processor type found." << std::endl;
    return nullptr;
}

// Register in the SensorFactory
const bool WOLF_UNUSED registered_prc_odom_3D = ProcessorParamsFactory::get().registerCreator("ODOM 3D", createProcessorOdom3DParams);

} // namespace [unnamed]

} // namespace wolf

