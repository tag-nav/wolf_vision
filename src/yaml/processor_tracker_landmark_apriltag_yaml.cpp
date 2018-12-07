/**
 * \file processor_tracker_landmark_apriltag_yaml.cpp
 *
 *  Created on: Dec 6, 2018
 *      \author: jsola
 */


// wolf
#include "../processors/processor_tracker_landmark_apriltag.h"
#include "yaml_conversion.h"
#include "../factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{

namespace
{
static ProcessorParamsBasePtr createProcessorParamsLandmarkApriltag(const std::string & _filename_dot_yaml)
{
    YAML::Node config = YAML::LoadFile(_filename_dot_yaml);

    if (config.IsNull())
    {
        WOLF_ERROR("Invalid YAML file!");
        return nullptr;
    }
    else if (config["processor type"].as<std::string>() == "TRACKER LANDMARK APRILTAG")
    {
        ProcessorParamsTrackerLandmarkApriltagPtr params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();

        params->type                        = config["processor type"]              .as<std::string>();
        params->name                        = config["processor name"]              .as<std::string>();

        YAML::Node detector_parameters      = config["detector parameters"];
        params->quad_decimate_              = detector_parameters["quad_decimate"]  .as<Scalar>();
        params->quad_sigma_                 = detector_parameters["quad_sigma"]     .as<Scalar>();
        params->nthreads_                   = detector_parameters["nthreads"]       .as<int>();
        params->debug_                      = detector_parameters["debug"]          .as<bool>();
        params->refine_edges_               = detector_parameters["refine_edges"]   .as<bool>();
        params->refine_decode_              = detector_parameters["refine_decode"]  .as<bool>();
        params->refine_pose_                = detector_parameters["refine_pose"]    .as<bool>();

        YAML::Node tag_parameters           = config["tag parameters"];
        params->tag_family_                 = tag_parameters["tag_family"]          .as<std::string>();
        params->tag_black_border_           = tag_parameters["tag_black_border"]    .as<int>();
        params->tag_width_default_          = tag_parameters["tag_width_default"]   .as<Scalar>();

        // read list of tag widths
        YAML::Node tag_widths               = config["tag widths"];
        for (auto pair_id_width : tag_widths)
        {
            int tag_id                      = pair_id_width.first                   .as<int>();
            Scalar tag_width                = pair_id_width.second                  .as<Scalar>();
            params->tag_widths_.emplace(tag_id, tag_width);
        }

        YAML::Node noise                    = config["noise"];
        params->std_xy_                      = noise["std_xy"]                       .as<Scalar>();
        params->std_z_                       = noise["std_z"]                        .as<Scalar>();
        params->std_rpy_           = M_TORAD * noise["std_rpy"]                      .as<Scalar>();

        return params;
    }
    else
    {
        WOLF_ERROR("Wrong processor type! Should be \"TRACKER LANDMARK APRILTAG\"");
        return nullptr;
    }
    return nullptr;
}

// Register in the SensorFactory
const bool WOLF_UNUSED registered_prc_apriltag = ProcessorParamsFactory::get().registerCreator("TRACKER LANDMARK APRILTAG", createProcessorParamsLandmarkApriltag);
const bool WOLF_UNUSED registered_prc_apriltag_wrapper = ProcessorParamsFactory::get().registerCreator("TRACKER LANDMARK APRILTAG WRAPPER", createProcessorParamsLandmarkApriltag);

} // namespace [unnamed]

} // namespace wolf
