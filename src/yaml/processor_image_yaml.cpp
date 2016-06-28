/**
 * \file processor_image_yaml.cpp
 *
 *  Created on: May 21, 2016
 *      \author: jsola
 */

// wolf yaml
#include "yaml_conversion.h"

// wolf
#include "../processor_image.h"
#include "../factory.h"

// yaml-cpp library
#include <yaml-cpp/yaml.h>

namespace wolf
{
namespace
{
static ProcessorParamsBase* createProcessorParamsImage(const std::string & _filename_dot_yaml)
{
    using std::string;
    using YAML::Node;

    ProcessorImageParameters* p = new ProcessorImageParameters;

    Node params = YAML::LoadFile(_filename_dot_yaml);

    std::string processor_type = params["processor type"].as<std::string>();
    std::cout << "processor type:: " << processor_type << std::endl;

    if (params["processor type"])
    {
        Node dd_yaml = params["detector-descriptor"];
        if(dd_yaml["type"].as<string>() == "ORB")
        {
            DetectorDescriptorParamsOrb* dd = new DetectorDescriptorParamsOrb;
            dd->type                     = DD_ORB;
            dd->nominal_pattern_radius   = dd_yaml["nominal pattern radius"].as<unsigned int>();
            dd->nfeatures                = dd_yaml["nfeatures"].as<unsigned int>();
            dd->scaleFactor              = dd_yaml["scale factor"].as<float>();
            dd->nlevels                  = dd_yaml["nlevels"].as<unsigned int>();
            dd->edgeThreshold            = dd_yaml["edge threshold"].as<unsigned int>();
            dd->firstLevel               = dd_yaml["first level"].as<unsigned int>();
            dd->WTA_K                    = dd_yaml["WTA_K"].as<unsigned int>();
            string st = dd_yaml["score type"].as<string>();
            if (st == "cv::ORB::HARRIS_SCORE")
                dd->scoreType            = cv::ORB::HARRIS_SCORE;
            else
            {
                std::cout << "Unknown score type" << std::endl;
            }
            dd->patchSize                = dd_yaml["patch size"].as<unsigned int>();
            p->detector_descriptor_params_ptr = dd;
        }else
        {
            std::cout << "Unknown detector-descriptor type " << dd_yaml["type"].as<string>() << std::endl;
            // TODO: add BRISK params struct
        }

        Node m = params["matcher"];
        p->matcher.min_normalized_score = m["minimum normalized score"].as<Scalar>();
        string sn = m["similarity norm"].as<string>();
        if(sn == "cv::NORM_HAMMING")
            p->matcher.similarity_norm  = cv::NORM_HAMMING;
        else
        {
            std::cout << "Unknown distance type" << std::endl;
        }
        p->matcher.roi_width            = m["roi"]["width"].as<unsigned int>();
        p->matcher.roi_height           = m["roi"]["height"].as<unsigned int>();

        Node as = params["active search"];
        p->active_search.grid_width     = as["grid width"].as<unsigned int>();
        p->active_search.grid_height    = as["grid height"].as<unsigned int>();
        p->active_search.separation     = as["separation"].as<unsigned int>();

        Node img = params["image"];
        p->image.width  = img["width"].as<unsigned int>();
        p->image.height = img["height"].as<unsigned int>();

        Node alg = params["algorithm"];
        p->algorithm.max_new_features = alg["maximum new features"].as<unsigned int>();
        p->algorithm.min_features_for_keyframe = alg["minimum features for new keyframe"].as<unsigned int>();

    }

    return p;
}

// Register in the SensorFactory
const bool registered_prc_image_par = ProcessorParamsFactory::get().registerCreator("IMAGE LANDMARK", createProcessorParamsImage);


}
}

