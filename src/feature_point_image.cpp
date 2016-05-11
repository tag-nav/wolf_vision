
#include "feature_point_image.h"

namespace wolf {

FeaturePointImage::FeaturePointImage(const Eigen::Vector2s & _measurement) :
    FeatureBase(FEATURE_POINT_IMAGE, _measurement,Eigen::MatrixXs::Zero(0,0)), is_known_(false)
{
    setType("POINT IMAGE");
}

FeaturePointImage::~FeaturePointImage()
{
    //
}

} // namespace wolf
