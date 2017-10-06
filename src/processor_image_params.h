#ifndef PROCESSOR_IMAGE_PARAMS_H
#define PROCESSOR_IMAGE_PARAMS_H

// Vision utils
#include <vision_utils.h>
#include <vision_utils/detectors.h>
#include <vision_utils/descriptors.h>
#include <vision_utils/matchers.h>

namespace wolf
{

struct ProcessorParamsImage : public ProcessorParamsBase
{
		std::string yaml_file_params_vision_utils;

        struct Algorithm
        {
                unsigned int max_new_features; ///< Max nbr. of features to detect in one frame
                unsigned int min_features_for_keyframe; ///< minimum nbr. of features to vote for keyframe
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
