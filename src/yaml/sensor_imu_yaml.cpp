/**
 * \file sensor_imu_yaml.cpp
 *
 *  Created on: Jan 18, 2017
 *      \author: Dinesh Atchuthan
 */

// wolf yaml
#include "yaml_conversion.h"

// wolf
#include "../sensor_imu.h"
#include "../factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{

namespace
{

static IntrinsicsBasePtr createIntrinsicsIMU(const std::string & _filename_dot_yaml)
{
    YAML::Node config = YAML::LoadFile(_filename_dot_yaml);

    if (config["sensor type"].as<std::string>() == "IMU")
    {

        // YAML:: to Eigen::
        using namespace Eigen;
        std::string sensor_type = config["sensor type"]     .as<std::string>();
        std::string sensor_name = config["sensor name"]     .as<std::string>();

        YAML::Node variances    = config["motion variances"];
        YAML::Node kf_vote      = config["keyframe vote"];

        IntrinsicsIMUPtr params = std::make_shared<IntrinsicsIMU>();

        params->a_noise     = variances["a_noise"]  .as<Scalar>();
        params->w_noise      = variances["w_noise"]   .as<Scalar>();
        params->ab_initial_stdev     = variances["ab_initial_stdev"]     .as<Scalar>();
        params->wb_initial_stdev     = variances["wb_initial_stdev"]     .as<Scalar>();
        params->ab_rate_stdev        = variances["ab_rate_stdev"]     .as<Scalar>();
        params->wb_rate_stdev        = variances["wb_rate_stdev"]     .as<Scalar>();

        return params;
    }

    std::cout << "Bad configuration file. No sensor type found." << std::endl;
    return nullptr;
}

// Register in the SensorFactory
const bool WOLF_UNUSED registered_imu_intr = IntrinsicsFactory::get().registerCreator("IMU", createIntrinsicsIMU);

} // namespace [unnamed]

} // namespace wolf
