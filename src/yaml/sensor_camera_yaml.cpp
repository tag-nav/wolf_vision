/**
 * \file sensor_camera_yaml.cpp
 *
 *  Created on: May 13, 2016
 *      \author: jsola
 */

// wolf yaml
#include "yaml_conversion.h"

// wolf
#include "../sensor_camera.h"
#include "../factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{

namespace
{
static IntrinsicsBase* createIntrinsicsCamera(const std::string & _filename_dot_yaml)
{
    YAML::Node camera_config = YAML::LoadFile(_filename_dot_yaml);

    if ("CAMERA") //camera_config["sensor type"])
    {

        // YAML:: to Eigen::
        using namespace Eigen;
        std::string sensor_type = "CAMERA"; //camera_config["sensor type"]                  .as<std::string>();
        std::string sensor_name = camera_config["camera_name"]                      .as<std::string>();
        unsigned int width      = camera_config["image_width"]                      .as<unsigned int>();
        unsigned int height     = camera_config["image_height"]                     .as<unsigned int>();
        VectorXd intrinsic      = camera_config["camera_matrix"]["data"]            .as<VectorXd>();
        VectorXd distortion     = camera_config["distortion_coefficients"]["data"]  .as<VectorXd>();

        // Eigen:: to wolf::
        IntrinsicsCamera* intrinsics_cam = new IntrinsicsCamera;
        intrinsics_cam->type = sensor_type;
        intrinsics_cam->name = sensor_name;
        intrinsics_cam->pinhole_model[0] = intrinsic[2];
        intrinsics_cam->pinhole_model[1] = intrinsic[5];
        intrinsics_cam->pinhole_model[2] = intrinsic[0];
        intrinsics_cam->pinhole_model[3] = intrinsic[4];
        intrinsics_cam->distortion = distortion;
        intrinsics_cam->width = width;
        intrinsics_cam->height = height;


        //=========================================
        // ===== this part for debugging only =====
        //=========================================
        std::cout << "\n--- Parameters Parsed from YAML file ---" << std::endl;
        std::cout << "sensor type: " << sensor_type << std::endl;
        std::cout << "sensor name: " << sensor_name << std::endl;

//        // extrinsics discarded in this creator
//        Vector3d pos            = camera_config["extrinsic"]["position"].as<Vector3d>();
//        Vector3d ori            = camera_config["extrinsic"]["orientation"].as<Vector3d>() * M_PI / 180; // roll, pitch, yaw [rad]
//        Quaternions quat; v2q(ori, quat);
//        std::cout << "sensor extrinsics: " << std::endl;
//        std::cout << "\tposition    : " << pos.transpose() << std::endl;
//        std::cout << "\torientation : " << ori.transpose() << std::endl;

        std::cout << "sensor intrinsics: " << std::endl;
        std::cout << "\timage size  : " << width << "x" << height << std::endl;
        std::cout << "\tintrinsic   : " << intrinsic.transpose() << std::endl;
        std::cout << "\tdistoriton  : " << distortion.transpose() << std::endl;
        //=========================================
        //=========================================



        return intrinsics_cam;
    }

    std::cout << "Bad configuration file. No sensor type found." << std::endl;
    return nullptr;
}

// Register in the SensorFactory
const bool registered_camera_intr = IntrinsicsFactory::get().registerCreator("CAMERA", createIntrinsicsCamera);

} // namespace [unnamed]

} // namespace wolf

