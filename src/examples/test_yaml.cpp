/**
 * \file yaml-test.cpp
 *
 *  Created on: May 1, 2016
 *      \author: jsola
 */

#include "pinholeTools.h"
#include "../yaml/yaml_conversion.h"

#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <fstream>


int main()
{

    YAML::Node camera_config = YAML::LoadFile("/home/jsola/dev/wolf/src/examples/camera.yaml");

    if (camera_config["sensor type"])
    {
        std::string sensor_type = camera_config["sensor type"].as<std::string>();

        std::string sensor_name = camera_config["sensor name"].as<std::string>();

        YAML::Node params = camera_config["intrinsic"];

        // convert yaml to Eigen
        using namespace Eigen;
        Vector3s pos = camera_config["extrinsic"]["position"].as<Vector3s>();
        Vector3s ori = camera_config["extrinsic"]["orientation"].as<Vector3s>() * M_PI / 180;
        Vector2s size = params["image size"].as<Vector2s>();
        Vector4s intrinsic = params["pinhole model"].as<Vector4s>();
        VectorXs distortion = params["distortion"].as<VectorXs>();

        // compute correction model
        VectorXs correction(distortion.size());
        pinhole::computeCorrectionModel(intrinsic, distortion, correction);

        // output
        std::cout << "sensor type       : " << sensor_type << std::endl;
        std::cout << "sensor name       : " << sensor_name << std::endl;
        std::cout << "sensor extrinsics : " << std::endl;
        std::cout << "\tposition        : " << pos.transpose() << std::endl;
        std::cout << "\torientation     : " << ori.transpose() << std::endl;
        std::cout << "sensor parameters : " << std::endl;
        std::cout << "\timage size      : " << size.transpose() << std::endl;
        std::cout << "\tpinhole model   : " << intrinsic.transpose() << std::endl;
        std::cout << "\tdistoriton      : " << distortion.transpose() << std::endl;
        std::cout << "\tcorrection      : " << correction.transpose() << std::endl;
    }
    else
        std::cout << "Bad configuration file. No sensor type found." << std::endl;

    return 0;
}
