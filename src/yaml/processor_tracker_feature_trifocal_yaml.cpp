/*
 * processor_tracker_feature_trifocal_yaml.cpp
 *
 *  Created on: Apr 12, 2018
 *      Author: asantamaria
 */

// wolf yaml
#include "vision/processor/processor_tracker_feature_trifocal.h"
#include "base/yaml/yaml_conversion.h"

// wolf
#include "base/factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{

namespace
{
static ProcessorParamsBasePtr createProcessorParamsTrackerFeatureTrifocal(const std::string & _filename_dot_yaml)
{
    YAML::Node config = YAML::LoadFile(_filename_dot_yaml);

    if (config.IsNull())
    {
        WOLF_ERROR("Invalid YAML file!");
        return nullptr;
    }
    else if (config["processor type"].as<std::string>() == "TRACKER FEATURE TRIFOCAL")
    {
        ProcessorParamsTrackerFeatureTrifocalPtr params = std::make_shared<ProcessorParamsTrackerFeatureTrifocal>();

        YAML::Node vision_utils               = config      ["vision_utils"];
        params->yaml_file_params_vision_utils = vision_utils["YAML file params"].as<std::string>();

        // relative to global path for Vision Utils YAML
        assert(params->yaml_file_params_vision_utils.at(0) != ('/') && "The parameter YAML FILE PARAMS (in processor params YAML file) must be specified with a path relative to the processor YAML file.");
        unsigned first = _filename_dot_yaml.find("/");
        unsigned last = _filename_dot_yaml.find_last_of("/");
        std::string strNew = _filename_dot_yaml.substr (first,last-first);
        params->yaml_file_params_vision_utils = _filename_dot_yaml.substr (first,last-first) + "/" + params->yaml_file_params_vision_utils;

        YAML::Node algorithm                    = config   ["algorithm"];
        params->time_tolerance                  = algorithm["time tolerance"]                 .as<Scalar>();
        params->voting_active                   = algorithm["voting active"]                  .as<bool>();
        params->min_features_for_keyframe       = algorithm["minimum features for keyframe"]  .as<unsigned int>();
        params->max_new_features                = algorithm["maximum new features"]           .as<unsigned int>();
        params->n_cells_h                       = algorithm["grid horiz cells"]               .as<int>();
        params->n_cells_v                       = algorithm["grid vert cells"]                .as<int>();
        params->min_response_new_feature        = algorithm["min response new features"]      .as<int>();
        params->max_euclidean_distance          = algorithm["max euclidean distance"]         .as<Scalar>();
        params->min_track_length_for_factor = algorithm["min track length for factor"].as<int>();

        YAML::Node noise                      = config["noise"];
        params->pixel_noise_std               = noise ["pixel noise std"].as<Scalar>();

        return params;
    }
    else
    {
        WOLF_ERROR("Wrong processor type! Should be \"TRACKER FEATURE TRIFOCAL\"");
        return nullptr;
    }
    return nullptr;
}

// Register in the SensorFactory
const bool WOLF_UNUSED registered_prc_trifocal = ProcessorParamsFactory::get().registerCreator("TRACKER FEATURE TRIFOCAL", createProcessorParamsTrackerFeatureTrifocal);

} // namespace [unnamed]

} // namespace wolf
