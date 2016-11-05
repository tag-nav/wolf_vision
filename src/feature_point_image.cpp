
#include "feature_point_image.h"

namespace wolf {

FeaturePointImage::FeaturePointImage(const Eigen::Vector2s & _measurement) :
    FeatureBase("POINT IMAGE", _measurement,Eigen::MatrixXs::Zero(0,0)), is_known_(false)
{
    //
}

FeaturePointImage::~FeaturePointImage()
{
    //
}

} // namespace wolf
