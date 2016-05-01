/**
 * \file yaml-test.cpp
 *
 *  Created on: May 1, 2016
 *      \author: jsola
 */

#include "yaml-cpp/yaml.h"

#include <iostream>
#include <fstream>

#include <eigen3/Eigen/Dense>

int main(){

    YAML::Node camera_config = YAML::LoadFile("/Users/jsola/dev/wolf/src/examples/camera.yaml");
    std::string sensor_type = "NULL";
    if (camera_config["sensor type"]) {
        sensor_type = camera_config["sensor type"].as<std::string>();

        YAML::Node params = camera_config["parameters"];

        std::vector<double> s = params["image size"].as<std::vector<double>>();
        std::vector<double> p = params["position"].as<std::vector<double>>();
        std::vector<double> o = params["orientation"].as<std::vector<double>>();
        std::vector<double> k = params["intrinsic"].as<std::vector<double>>();

        using namespace Eigen;

        Map<Vector2d> size(s.data());
        Map<Vector3d> pos(p.data());
        Map<Vector3d> ori(o.data());
        ori *= 3.141592653/180;
        Map<Vector4d> intrinsic(k.data());

        std::cout << "sensor type: " << sensor_type << std::endl;
        std::cout << "position   : " << pos.transpose() << std::endl;
        std::cout << "orientation: " << ori.transpose() << std::endl;
        std::cout << "intrinsic  : " << intrinsic.transpose() << std::endl;
        std::cout << "image size : " << size.transpose() << std::endl;
    }
    else
        std::cout << "Bad configuration file. No sensor type found." << std::endl;

    return 0;
}
