
#include "feature_corner_2D.h"

FeatureCorner2D::FeatureCorner2D(const Eigen::Vector2s & _measurement, const Eigen::Matrix2s & _meas_covariance) :
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
