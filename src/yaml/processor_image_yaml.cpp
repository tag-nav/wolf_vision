/**
 * \file processor_image_yaml.cpp
 *
 *  Created on: May 21, 2016
 *      \author: jsola
 */

#include "vision/internal/config.h"
#include "vision/processor/processor_params_image.h"

// wolf yaml
#include "core/yaml/yaml_conversion.h"

// wolf
#include "core/common/factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>


namespace wolf
{
namespace
{
static ParamsProcessorBasePtr createParamsProcessorImage(const std::string & _filename_dot_yaml)
{
    using std::string;
    using YAML::Node;

    std::shared_ptr<ParamsProcessorTrackerFeatureImage> p = std::make_shared<ParamsProcessorTrackerFeatureImage>();

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
   	            std::string wolf_root = _WOLF_VISION_ROOT_DIR;
   	            std::cout << "Wolf root: " << wolf_root << std::endl;
   	            std::string abs_path = wolf_root + "/demos/" + p->yaml_file_params_vision_utils;
   	            p->yaml_file_params_vision_utils = abs_path;
   	        }
   	    }

        Node alg = params["algorithm"];
        p->max_new_features = alg["maximum new features"].as<unsigned int>();
        p->min_features_for_keyframe = alg["minimum features for new keyframe"].as<unsigned int>();
        p->min_response_for_new_features = alg["minimum response for new features"].as<float>();
        p->time_tolerance= alg["time tolerance"].as<double>();
        p->distance= alg["distance"].as<double>();

        Node noi = params["noise"];
        p->pixel_noise_std = noi["pixel noise std"].as<double>();
        p->pixel_noise_var = p->pixel_noise_std * p->pixel_noise_std;
    }

    return p;
}

// Register in the FactorySensor
const bool WOLF_UNUSED registered_prc_image_feature_par = FactoryParamsProcessor::registerCreator("ProcessorTrackerFeatureImage", createParamsProcessorImage);
const bool WOLF_UNUSED registered_prc_image_landmark_par = FactoryParamsProcessor::registerCreator("ProcessorTrackerLandmarkImage", createParamsProcessorImage);

}
}
