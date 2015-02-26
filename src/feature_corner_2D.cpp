
#include "feature_corner_2D.h"

FeatureCorner2D::FeatureCorner2D(const Eigen::Vector4s & _measurement, const Eigen::Matrix4s & _meas_covariance) :
    FeatureBase(_measurement, _meas_covariance)
{
    //
}

FeatureCorner2D::~FeatureCorner2D()
{
    //
}

void FeatureCorner2D::findConstraints()
{
    //
}
