//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
// wolf yaml
#include "core/yaml/yaml_conversion.h"

// wolf
#include "core/common/factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>
#include "vision/processor/processor_visual_odometry.h"

namespace wolf
{

namespace
{
static ParamsProcessorBasePtr createParamsProcessorVisualOdometry(const std::string & _filename_dot_yaml)
{
    YAML::Node config = YAML::LoadFile(_filename_dot_yaml);

    if (config.IsNull())
    {
        WOLF_ERROR("Invalid YAML file!");
        return nullptr;
    }
    else if (config["type"].as<std::string>() == "ProcessorVisualOdometry")
    {
        ParamsProcessorVisualOdometryPtr params = std::make_shared<ParamsProcessorVisualOdometry>();

        // YAML::Node algorithm                    = config   ["algorithm"];
        // params->time_tolerance                  = algorithm["time tolerance"]                 .as<double>();
        // params->voting_active                   = algorithm["voting active"]                  .as<bool>();
        // params->delete_ambiguities              = algorithm["delete ambiguities"]             .as<bool>();
        // params->min_features_for_keyframe       = algorithm["minimum features for keyframe"]  .as<unsigned int>();
        // params->max_new_features                = algorithm["maximum new features"]           .as<unsigned int>();

        // params->n_cells_h                       = algorithm["grid horiz cells"]               .as<int>();
        // params->n_cells_v                       = algorithm["grid vert cells"]                .as<int>();
        // params->min_response_new_feature        = algorithm["min response new features"]      .as<int>();

        // params->min_track_length_for_factor = algorithm["min track length for factor"].as<int>();


        return params;
    }
    else
    {
        WOLF_ERROR("Wrong processor type! Should be \"TRACKER BUNDLE ADJUSTMENT\"");
        return nullptr;
    }
    return nullptr;
}

// Register in the FactorySensor
const bool WOLF_UNUSED registered_prc_bundle_adjustment = FactoryParamsProcessor::registerCreator("ProcessorVisualOdometry", createParamsProcessorVisualOdometry);

} // namespace [unnamed]

} // namespace wolf
