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

int main()
{

    YAML::Node camera_config = YAML::LoadFile("/home/jsola/dev/wolf/src/examples/camera.yaml");

    if (camera_config["sensor type"])
    {
        std::string sensor_type = camera_config["sensor type"].as<std::string>();

        std::string sensor_name = camera_config["sensor name"].as<std::string>();

        YAML::Node params = camera_config["parameters"];

        std::vector<double> p = camera_config["extrinsic"]["position"].as<std::vector<double> >(); // in one go: it works!
        std::vector<double> o = camera_config["extrinsic"]["orientation"].as<std::vector<double> >();
        std::vector<double> s = params["image size"].as<std::vector<double> >();
        std::vector<double> k = params["intrinsic"].as<std::vector<double> >();
        std::vector<double> d = params["distortion"].as<std::vector<double> >();
        std::vector<double> c = params["correction"].as<std::vector<double> >();

        using namespace Eigen;

        // Using Eigen vector constructors from data pionters. Mind the vector sizes!
        Vector3d pos(p.data());
        Vector3d ori(o.data());
        ori *= 3.1415926536 / 180;
        Vector2d size(s.data());
        Vector4d intrinsic(k.data());
        Map<VectorXd> distortion(d.data(), d.size());
        Map<VectorXd> correction(c.data(), c.size());

        std::cout << "sensor type: " << sensor_type << std::endl;
        std::cout << "sensor name: " << sensor_name << std::endl;
        std::cout << "sensor extrinsics: " << std::endl;
        std::cout << "\tposition    : " << pos.transpose() << std::endl;
        std::cout << "\torientation : " << ori.transpose() << std::endl;
        std::cout << "sensor parameters: " << std::endl;
        std::cout << "\timage size  : " << size.transpose() << std::endl;
        std::cout << "\tintrinsic   : " << intrinsic.transpose() << std::endl;
        std::cout << "\tdistoriton  : " << distortion.transpose() << std::endl;
        std::cout << "\tcorrection  : " << correction.transpose() << std::endl;
    }
    else
        std::cout << "Bad configuration file. No sensor type found." << std::endl;

    return 0;
}
