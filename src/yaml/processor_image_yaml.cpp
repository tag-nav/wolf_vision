/**
 * \file processor_image_yaml.cpp
 *
 *  Created on: May 21, 2016
 *      \author: jsola
 */

// wolf yaml
#include "yaml_conversion.h"

// wolf
#include "../processor_params_image.h"
#include "../factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{
namespace
{
static ProcessorParamsBasePtr createProcessorParamsImage(const std::string & _filename_dot_yaml)
{
    using std::string;
    using YAML::Node;

    std::shared_ptr<ProcessorParamsImage> p = std::make_shared<ProcessorParamsImage>();

    Node params = YAML::LoadFile(_filename_dot_yaml);

    if (!params.IsNull())
    {
    	Node dd_yaml = params["vision_utils"];
   	    p->yaml_file_params_vision_utils = dd_yaml["YAML file params"].as<std::string>();
   	    // Check if relative path
   	    if (!p->yaml_file_params_vision_utils.empty())
   	    {
   	        if (p->yaml_file_params_vision_utils[0] != '/')
   	        {
   	            std::string wolf_root = _WOLF_ROOT_DIR;
   	            std::cout << "Wolf root: " << wolf_root << std::endl;
   	            std::string abs_path = wolf_root + "/src/examples/" + p->yaml_file_params_vision_utils;
   	            p->yaml_file_params_vision_utils = abs_path;
   	        }
   	    }

        Node alg = params["algorithm"];
        p->algorithm.max_new_features = alg["maximum new features"].as<unsigned int>();
        p->algorithm.min_features_for_keyframe = alg["minimum features for new keyframe"].as<unsigned int>();
        p->algorithm.min_response_for_new_features = alg["minimum response for new features"].as<float>();
        p->algorithm.time_tolerance= alg["time tolerance"].as<Scalar>();
        p->algorithm.distance= alg["distance"].as<Scalar>();

        Node noi = params["noise"];
        p->noise.pixel_noise_std = noi["pixel noise std"].as<Scalar>();
        p->noise.pixel_noise_var = p->noise.pixel_noise_std * p->noise.pixel_noise_std;
    }

    return p;
}

// Register in the SensorFactory
const bool WOLF_UNUSED registered_prc_image_feature_par = ProcessorParamsFactory::get().registerCreator("IMAGE FEATURE", createProcessorParamsImage);
const bool WOLF_UNUSED registered_prc_image_landmark_par = ProcessorParamsFactory::get().registerCreator("IMAGE LANDMARK", createProcessorParamsImage);


}
}

