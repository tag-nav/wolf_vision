
#include "feature_point.h"

/**
 *
 * Test for the feature image
 *
 **/

FeaturePoint::FeaturePoint(const Eigen::Vector2s & _measurement) :
    FeatureBase(_measurement,Eigen::MatrixXs::Zero(0,0)), measurement_(_measurement)
{
    //
}

FeaturePoint::FeaturePoint(const Eigen::Vector2s & _measurement, const Eigen::Matrix2s & _meas_covariance) :
    FeatureBase(_measurement, _meas_covariance), measurement_(_measurement)
{
    //
}

FeaturePoint::~FeaturePoint()
{
    //
}

