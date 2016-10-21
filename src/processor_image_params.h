#ifndef PROCESSOR_IMAGE_PARAMS_H
#define PROCESSOR_IMAGE_PARAMS_H

namespace wolf
{

enum DetectorDescriptorType
{
    DD_BRISK,
    DD_ORB
};

struct DetectorDescriptorParamsBase
{
        DetectorDescriptorType type; ///< Type of algorithm. Accepted values in wolf.h
};

struct DetectorDescriptorParamsBrisk : public DetectorDescriptorParamsBase
{
        unsigned int threshold=30; ///< on the keypoint strength to declare it key-point
        unsigned int octaves=0; ///< Multi-scale evaluation. 0: no multi-scale
        float pattern_scale=1.0f; ///< Scale of the base pattern wrt the nominal one
        unsigned int nominal_pattern_radius = 18; ///< Radius of the pattern before scaling //18 for brisk
};

struct DetectorDescriptorParamsOrb : public DetectorDescriptorParamsBase
{
        unsigned int nfeatures=500; ///< Nbr of features to extract
        float scaleFactor=1.2f; ///< Scale factor between two consecutive scales of the image pyramid
        unsigned int nlevels=8;///< Number of levels in the pyramid. Default: 8
        unsigned int edgeThreshold=16; ///< Default: 16
        unsigned int firstLevel=0;
        unsigned int WTA_K=2;
        unsigned int scoreType=cv::ORB::HARRIS_SCORE; ///< Type of score to rank the detected points
        unsigned int patchSize=31;
};

struct ProcessorParamsImage : public ProcessorParamsBase
{
        DetectorDescriptorParamsBase* detector_descriptor_params_ptr;

        struct Matcher
        {
                Scalar min_normalized_score; ///< [-1..0]: awful match; 1: perfect match; out of [-1,1]: error
                int similarity_norm; ///< Norm used to measure the distance between two descriptors
                unsigned int roi_width; ///< Width of the roi used in tracking
                unsigned int roi_height; ///< Height of the roi used in tracking
        }matcher;
        struct Active_search
        {
                unsigned int grid_width; ///< cells per horizontal dimension of image
                unsigned int grid_height; ///< cells per vertical dimension of image
                unsigned int separation; ///< Distance between the border of the cell and the border of the associated ROI
        }active_search;
        struct Algorithm
        {
                unsigned int max_new_features; ///< Max nbr. of features to detect in one frame
                unsigned int min_features_for_keyframe; ///< minimum nbr. of features to vote for keyframe
                float min_response_for_new_features; ///< minimum value of the response to create a new feature
        }algorithm;

        struct Draw
        {
                bool primary_drawing; ///< draw the features found in the image
                bool secondary_drawing; ///< draw the target of the features found in the image
                bool detector_roi; ///< draw the roi in which new features are searched
                bool tracker_roi; ///< draw the roi used to track features from the last frame
        }draw;
};
}

#endif // PROCESSOR_IMAGE_PARAMS_H
