
#include "feature_point_image.h"

namespace wolf {

FeaturePointImage::FeaturePointImage(const Eigen::Vector2s & _measurement) :
    FeatureBase("POINT IMAGE", _measurement,Eigen::MatrixXs::Zero(0,0)), is_known_(false)
{
    keypoint_.pt.x = float(measurement_(0));
    keypoint_.pt.y = float(measurement_(1));
    //
}

FeaturePointImage::~FeaturePointImage()
{
    //
}

} // namespace wolf
