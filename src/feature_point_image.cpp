
#include "feature_point_image.h"

/**
 *
 * Test for the feature image
 *
 **/

FeaturePointImage::FeaturePointImage(const Eigen::Vector2s & _measurement) :
    FeatureBase(FEAT_POINT_IMAGE, _measurement,Eigen::MatrixXs::Zero(0,0)), measurement_(_measurement)
{
    //
}

FeaturePointImage::FeaturePointImage(const Eigen::Vector2s & _measurement, const cv::KeyPoint _keypoint ,const std::vector<float> & _descriptor):
    FeatureBase(FEAT_POINT_IMAGE, _measurement,Eigen::MatrixXs::Zero(0,0)), measurement_(_measurement), keypoint_(_keypoint),descriptor_(_descriptor)
{

}

FeaturePointImage::FeaturePointImage(const Eigen::Vector2s & _measurement, const Eigen::Matrix2s & _meas_covariance) :
    FeatureBase(FEAT_POINT_IMAGE, _measurement, _meas_covariance), measurement_(_measurement)
{
    //
}

FeaturePointImage::~FeaturePointImage()
{
    //
}

Eigen::Vector2s FeaturePointImage::getMeasurement()
{
    return measurement_;
}

cv::KeyPoint FeaturePointImage::getKeypoint()
{
    return keypoint_;
}

std::vector<float> FeaturePointImage::getDescriptor()
{
    return descriptor_;
}

