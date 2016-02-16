
#include "feature_point.h"

/**
 *
 * Test for the feature image
 *
 **/

FeaturePoint::FeaturePoint(const Eigen::Vector2s & _measurement, const Eigen::Matrix2s & _meas_covariance) :
    FeatureBase(_measurement, _meas_covariance)
{
    //
}

FeaturePoint::~FeaturePoint()
{
    //
}

