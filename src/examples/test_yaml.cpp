/**
 * \file yaml-test.cpp
 *
 *  Created on: May 1, 2016
 *      \author: jsola
 */

#include "pinholeTools.h"
#include "yaml/yaml_conversion.h"
#include "processor_image.h"
#include "factory.h"

#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Dense>

#include <iostream>
#include <fstream>


int main()
{

    /**=============================================================================================
     * Get wolf root directory from the environment variable WOLF_ROOT
     * To make this work, you need to set the environment variable WOLF_ROOT:
     *  - To run from terminal, edit your ~/.bashrc, or ~/.bash_profile and add this line:
     *        export WOLF_ROOT="/path/to/wolf"
     *  - To run from eclipse, open the 'run configuration' of this executable, tab 'Environment'
     *    and add variable WOLF_ROOT set to /path/to/wolf
     */
    std::string WOLF_ROOT;
    char* w = std::getenv("WOLF_ROOT");
    if (w != NULL)
        WOLF_ROOT = w;
    else
        throw std::runtime_error("Environment variable WOLF_ROOT not found");
    std::cout << "\nwolf root directory: " << WOLF_ROOT << std::endl;
    //=============================================================================================

    using namespace Eigen;
    using namespace wolf;
    using std::string;
    using YAML::Node;

    // Camera parameters

    YAML::Node camera_config = YAML::LoadFile(WOLF_ROOT + "/src/examples/camera.yaml");

    if (camera_config["sensor type"])
    {
        std::string sensor_type = camera_config["sensor type"].as<std::string>();

        std::string sensor_name = camera_config["sensor name"].as<std::string>();

        YAML::Node params   = camera_config["intrinsic"];

        // convert yaml to Eigen
        Vector3s pos        = camera_config["extrinsic"]["position"].as<Vector3s>();
        Vector3s ori        = camera_config["extrinsic"]["orientation"].as<Vector3s>() * M_PI / 180;
        Vector2s size       = params["image size"].as<Vector2s>();
        Vector4s intrinsic  = params["pinhole model"].as<Vector4s>();
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



    // Processor Image parameters

    ProcessorParamsImage p;

    Node params = YAML::LoadFile(WOLF_ROOT + "/src/examples/processor_image_ORB.yaml");

    if (params["processor type"])
    {
        Node dd_yaml = params["detector-descriptor"];
        if(dd_yaml["type"].as<string>() == "ORB")
        {
            DetectorDescriptorParamsOrb* dd = new DetectorDescriptorParamsOrb;
            dd->type                    = DD_ORB;
            dd->nominal_pattern_radius  = dd_yaml["nominal pattern radius"].as<unsigned int>();
            dd->nfeatures               = dd_yaml["nfeatures"].as<unsigned int>();
            dd->scaleFactor             = dd_yaml["scale factor"].as<float>();
            dd->nlevels                 = dd_yaml["nlevels"].as<unsigned int>();
            dd->edgeThreshold           = dd_yaml["edge threshold"].as<unsigned int>();
            dd->firstLevel              = dd_yaml["first level"].as<unsigned int>();
            dd->WTA_K                   = dd_yaml["WTA_K"].as<unsigned int>();
            dd->scoreType               = dd_yaml["score type"].as<int>(); // enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
            dd->patchSize               = dd_yaml["patch size"].as<unsigned int>();
            p.detector_descriptor_params_ptr = dd;
        }
        else
            std::cout << "Unknown detector-descriptor type " << dd_yaml["type"].as<string>() << std::endl;

        Node m = params["matcher"];
        p.matcher.min_normalized_score  = m["minimum normalized score"].as<Scalar>();
        p.matcher.similarity_norm       = m["similarity norm"].as<int>(); // enum { NORM_INF=1, NORM_L1=2, NORM_L2=4, NORM_L2SQR=5, NORM_HAMMING=6, NORM_HAMMING2=7, NORM_TYPE_MASK=7, NORM_RELATIVE=8, NORM_MINMAX=32 };

        p.matcher.roi_width             = m["roi"]["width"].as<unsigned int>();
        p.matcher.roi_height            = m["roi"]["height"].as<unsigned int>();

        Node as = params["active search"];
        p.active_search.grid_width      = as["grid width"].as<unsigned int>();
        p.active_search.grid_height     = as["grid height"].as<unsigned int>();
        p.active_search.separation      = as["separation"].as<unsigned int>();

        Node img = params["image"];
        p.image.width                   = img["width"].as<unsigned int>();
        p.image.height                  = img["height"].as<unsigned int>();

        Node alg = params["algorithm"];
        p.algorithm.max_new_features            = alg["maximum new features"].as<unsigned int>();
        p.algorithm.min_features_for_keyframe   = alg["minimum features for new keyframe"].as<unsigned int>();
    }


    return 0;
}
