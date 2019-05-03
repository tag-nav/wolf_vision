#ifndef PROCESSOR_IMAGE_PARAMS_H
#define PROCESSOR_IMAGE_PARAMS_H

// wolf
#include "core/processor/processor_tracker_feature.h"
#include "core/processor/processor_tracker_landmark.h"

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
};

WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsTrackerLandmarkImage);

struct ProcessorParamsTrackerLandmarkImage : public ProcessorParamsTrackerLandmark
{
        std::string yaml_file_params_vision_utils;

        Scalar min_response_for_new_features; ///< minimum value of the response to create a new feature
        Scalar distance;

        Scalar pixel_noise_std; ///< std noise of the pixel
        Scalar pixel_noise_var; ///< var noise of the pixel
};
}

#endif // PROCESSOR_IMAGE_PARAMS_H
