/**
 * \file processor_image_yaml.cpp
 *
 *  Created on: May 21, 2016
 *      \author: jsola
 */

// wolf yaml
#include "yaml_conversion.h"

// wolf
#include "../processor_image_feature.h"
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

        Node as = params["active search"];
        p->active_search.grid_width     = as["grid width"].as<unsigned int>();
        p->active_search.grid_height    = as["grid height"].as<unsigned int>();
        p->active_search.separation     = as["separation"].as<unsigned int>();

        Node alg = params["algorithm"];
        p->algorithm.max_new_features = alg["maximum new features"].as<unsigned int>();
        p->algorithm.min_features_for_keyframe = alg["minimum features for new keyframe"].as<unsigned int>();
        p->algorithm.min_response_for_new_features = alg["minimum response for new features"].as<float>();
        p->algorithm.time_tolerance= alg["time tolerance"].as<Scalar>();
        p->algorithm.distance= alg["distance"].as<Scalar>();

        Node noi = params["noise"];
        p->noise.pixel_noise_std = noi["pixel noise std"].as<Scalar>();
        p->noise.pixel_noise_var = p->noise.pixel_noise_std * p->noise.pixel_noise_std;

        Node draw = params["draw"];
        p->draw.primary_drawing = draw["primary draw"].as<bool>();
        p->draw.secondary_drawing = draw["secondary draw"].as<bool>();
        p->draw.detector_roi = draw["detection roi"].as<bool>();
        p->draw.tracker_roi = draw["tracking roi"].as<bool>();

    }

    return p;
}

// Register in the SensorFactory
const bool WOLF_UNUSED registered_prc_image_feature_par = ProcessorParamsFactory::get().registerCreator("IMAGE FEATURE", createProcessorParamsImage);
const bool WOLF_UNUSED registered_prc_image_landmark_par = ProcessorParamsFactory::get().registerCreator("IMAGE LANDMARK", createProcessorParamsImage);


}
}

