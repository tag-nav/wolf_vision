
#include "feature_corner_2D.h"

FeatureCorner2D::FeatureCorner2D(const Eigen::Vector3s & _measurement, const Eigen::Matrix3s & _meas_covariance) :
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
