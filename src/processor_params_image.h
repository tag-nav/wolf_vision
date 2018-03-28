#ifndef PROCESSOR_IMAGE_PARAMS_H
#define PROCESSOR_IMAGE_PARAMS_H

// wolf
#include "processor_base.h"
#include "wolf.h"

// Vision utils
#include <vision_utils.h>
#include <vision_utils/detectors.h>
#include <vision_utils/descriptors.h>
#include <vision_utils/matchers.h>

namespace wolf
{

struct ProcessorParamsImage : public ProcessorParamsTracker
{
		std::string yaml_file_params_vision_utils;

        struct Algorithm
        {
                float min_response_for_new_features; ///< minimum value of the response to create a new feature
                Scalar time_tolerance;
                Scalar distance;
        }algorithm;

        struct Noise
        {
                Scalar pixel_noise_std; ///< std noise of the pixel
                Scalar pixel_noise_var; ///< var noise of the pixel
        }noise;
};
}

#endif // PROCESSOR_IMAGE_PARAMS_H
