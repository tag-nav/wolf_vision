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

        YAML::Node variances = config["motion variances"];
        YAML::Node kf_vote = config["keyframe vote"];

        IntrinsicsIMUPtr params = std::make_shared<IntrinsicsIMU>();

        params->accel_noise   = variances["accel_noise"] .as<Scalar>();
        params->gyro_noise    = variances["gyro_noise"]  .as<Scalar>();
        params->ab_constr     = variances["ab_constr"]   .as<Scalar>();
        params->wb_constr     = variances["wb_constr"]   .as<Scalar>();

        return params;
    }

    std::cout << "Bad configuration file. No sensor type found." << std::endl;
    return nullptr;
}

// Register in the SensorFactory
const bool registered_imu_intr = IntrinsicsFactory::get().registerCreator("IMU", createIntrinsicsIMU);

} // namespace [unnamed]

} // namespace wolf