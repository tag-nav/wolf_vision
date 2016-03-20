
#include "feature_point_image.h"

/**
 *
 * Test for the feature image
 *
 **/

FeaturePointImage::FeaturePointImage(const Eigen::Vector2s & _measurement) :
    FeatureBase(FEAT_POINT_IMAGE, _measurement,Eigen::MatrixXs::Zero(0,0))
{
    //
}

FeaturePointImage::~FeaturePointImage()
{
    //
}
