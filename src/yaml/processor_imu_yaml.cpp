/**
 * \file processor_imu_yaml.cpp
 *
 *  Created on: jan 19, 2017
 *      \author: Dinesh Atchuthan
 */

// wolf yaml
#include <processor_IMU.h>
#include "yaml_conversion.h"

// wolf
#include "../factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{

namespace
{
static ProcessorParamsBasePtr createProcessorIMUParams(const std::string & _filename_dot_yaml)
{
    YAML::Node config = YAML::LoadFile(_filename_dot_yaml);

    if (config["processor type"].as<std::string>() == "IMU")
    {

        // YAML:: to Eigen::
        using namespace Eigen;
        std::string processor_type = config["processor type"]     .as<std::string>();
        std::string processor_name = config["processor name"]     .as<std::string>();

        YAML::Node kf_vote = config["keyframe vote"];

        ProcessorIMUParamsPtr params = std::make_shared<ProcessorIMUParams>();

        params->type                = processor_type;
        params->name                = processor_name;
        params->max_time_span       = kf_vote["max time span"]      .as<Scalar>();
        params->max_buff_length     = kf_vote["max buffer length"]  .as<Size  >();
        params->dist_traveled       = kf_vote["dist traveled"]      .as<Scalar>();
        params->angle_turned        = kf_vote["angle turned"]       .as<Scalar>();
        params->voting_active       = kf_vote["voting_active"]      .as<bool>();

        return params;
    }

    std::cout << "Bad configuration file. No processor type found." << std::endl;
    return nullptr;
}

// Register in the SensorFactory
const bool WOLF_UNUSED registered_prc_odom_3D = ProcessorParamsFactory::get().registerCreator("IMU", createProcessorIMUParams);

} // namespace [unnamed]

} // namespace wolf
