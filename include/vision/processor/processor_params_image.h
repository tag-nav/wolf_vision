#ifndef PROCESSOR_IMAGE_PARAMS_H
#define PROCESSOR_IMAGE_PARAMS_H

// wolf
#include "core/processor/processor_tracker_feature.h"
#include "core/processor/processor_tracker_landmark.h"
#include "core/utils/params_server.h"

namespace wolf
{

WOLF_STRUCT_PTR_TYPEDEFS(ParamsProcessorTrackerFeatureImage);

struct ParamsProcessorTrackerFeatureImage : public ParamsProcessorTrackerFeature
{
        std::string yaml_file_params_vision_utils;

        double min_response_for_new_features; ///< minimum value of the response to create a new feature
        double distance;

        double pixel_noise_std; ///< std noise of the pixel
        double pixel_noise_var; ///< var noise of the pixel
    ParamsProcessorTrackerFeatureImage() = default;
    ParamsProcessorTrackerFeatureImage(std::string _unique_name, const ParamsServer& _server):
        ParamsProcessorTrackerFeature(_unique_name, _server)
    {
        yaml_file_params_vision_utils   = _server.getParam<std::string>(_unique_name    + "/yaml_file_params_vision_utils");
        min_response_for_new_features   = _server.getParam<double>(_unique_name         + "/min_response_for_new_features");
        distance                        = _server.getParam<double>(_unique_name         + "/distance");
        pixel_noise_std                 = _server.getParam<double>(_unique_name         + "/pixel_noise_std");
        pixel_noise_var                 = _server.getParam<double>(_unique_name         + "/pixel_noise_var");
    }
    std::string print() const
    {
        return "\n" + ParamsProcessorTrackerFeature::print()                                    + "\n"
            + "yaml_file_params_vision_utils: " + yaml_file_params_vision_utils                 + "\n"
            + "min_response_for_new_features: " + std::to_string(min_response_for_new_features) + "\n"
            + "distance: "                      + std::to_string(distance)                      + "\n"
            + "pixel_noise_std: "               + std::to_string(pixel_noise_std)               + "\n"
            + "pixel_noise_var: "               + std::to_string(pixel_noise_var)               + "\n";
    }
};

WOLF_STRUCT_PTR_TYPEDEFS(ParamsProcessorTrackerLandmarkImage);

struct ParamsProcessorTrackerLandmarkImage : public ParamsProcessorTrackerLandmark
{
        std::string yaml_file_params_vision_utils;

        double min_response_for_new_features; ///< minimum value of the response to create a new feature
        double distance;

        double pixel_noise_std; ///< std noise of the pixel
        double pixel_noise_var; ///< var noise of the pixel

        ParamsProcessorTrackerLandmarkImage(std::string _unique_name, const ParamsServer& _server):
            ParamsProcessorTrackerLandmark(_unique_name, _server)
        {
            yaml_file_params_vision_utils   = _server.getParam<std::string>(_unique_name    + "/yaml_file_params_vision_utils");

            min_response_for_new_features   = _server.getParam<double>(_unique_name         + "/min_response_for_new_features");
            distance                        = _server.getParam<double>(_unique_name         + "/distance");

            pixel_noise_std                 = _server.getParam<double>(_unique_name         + "/pixel_noise_std");
            pixel_noise_var                 = _server.getParam<double>(_unique_name         + "/pixel_noise_var");
        }
        std::string print() const
        {
            return "\n" + ParamsProcessorTrackerLandmark::print()                                       + "\n"
                    + "yaml_file_params_vision_utils: " + yaml_file_params_vision_utils                 + "\n"
                    + "min_response_for_new_features: " + std::to_string(min_response_for_new_features) + "\n"
                    + "distance: "                      + std::to_string(distance)                      + "\n"
                    + "pixel_noise_std: "               + std::to_string(pixel_noise_std)               + "\n"
                    + "pixel_noise_var: "               + std::to_string(pixel_noise_var)               + "\n";
        }
};
}

#endif // PROCESSOR_IMAGE_PARAMS_H
