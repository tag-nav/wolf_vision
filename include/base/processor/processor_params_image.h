#ifndef PROCESSOR_IMAGE_PARAMS_H
#define PROCESSOR_IMAGE_PARAMS_H

// wolf
#include "base/processor/processor_tracker_feature.h"
#include "base/processor/processor_tracker_landmark.h"
#include "base/utils/params_server.hpp"

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsTrackerFeatureImage);

struct ProcessorParamsTrackerFeatureImage : public ProcessorParamsTrackerFeature
{
        std::string yaml_file_params_vision_utils;

        Scalar min_response_for_new_features; ///< minimum value of the response to create a new feature
        Scalar distance;

        Scalar pixel_noise_std; ///< std noise of the pixel
        Scalar pixel_noise_var; ///< var noise of the pixel
    ProcessorParamsTrackerFeatureImage() = default;
    ProcessorParamsTrackerFeatureImage(std::string _unique_name, const paramsServer& _server):
        ProcessorParamsTrackerFeature(_unique_name, _server)
    {
        yaml_file_params_vision_utils = _server.getParam<std::string>(_unique_name + "/yaml_file_params_vision_utils");
        min_response_for_new_features = _server.getParam<Scalar>(_unique_name + "/min_response_for_new_features");
        distance = _server.getParam<Scalar>(_unique_name + "/distance");
        pixel_noise_std = _server.getParam<Scalar>(_unique_name + "/pixel_noise_std");
        pixel_noise_var = _server.getParam<Scalar>(_unique_name + "/pixel_noise_var");
    }
};

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsTrackerLandmarkImage);

struct ProcessorParamsTrackerLandmarkImage : public ProcessorParamsTrackerLandmark
{
        std::string yaml_file_params_vision_utils;

        Scalar min_response_for_new_features; ///< minimum value of the response to create a new feature
        Scalar distance;

        Scalar pixel_noise_std; ///< std noise of the pixel
        Scalar pixel_noise_var; ///< var noise of the pixel
    ProcessorParamsTrackerLandmarkImage(std::string _unique_name, const paramsServer& _server):
        ProcessorParamsTrackerLandmark(_unique_name, _server)
    {
        yaml_file_params_vision_utils = _server.getParam<std::string>(_unique_name + "/yaml_file_params_vision_utils");

        min_response_for_new_features = _server.getParam<Scalar>(_unique_name + "/min_response_for_new_features");
        distance = _server.getParam<Scalar>(_unique_name + "/distance");

        pixel_noise_std = _server.getParam<Scalar>(_unique_name + "/pixel_noise_std");
        pixel_noise_var = _server.getParam<Scalar>(_unique_name + "/pixel_noise_var");
    }
};
}

#endif // PROCESSOR_IMAGE_PARAMS_H
