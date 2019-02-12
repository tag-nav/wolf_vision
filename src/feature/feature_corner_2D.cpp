
#include "base/feature/feature_corner_2D.h"

namespace wolf {

FeatureCorner2D::FeatureCorner2D(const Eigen::Vector4s & _measurement, const Eigen::Matrix4s & _meas_covariance) :
    FeatureBase("CORNER 2D", _measurement, _meas_covariance)
{
    //std::cout << "feature: "<< _measurement.transpose() << std::endl;
}

FeatureCorner2D::~FeatureCorner2D()
{
    //
}

Scalar FeatureCorner2D::getAperture() const
{
    return measurement_(3);
}

} // namespace wolf
