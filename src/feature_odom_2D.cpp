#include "feature_odom_2D.h"

FeatureOdom2D::FeatureOdom2D(unsigned int _dim_measurement) :
    FeatureBase(FEAT_ODOM_2D, _dim_measurement)
{
    //
}

FeatureOdom2D::FeatureOdom2D(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase(FEAT_ODOM_2D, _measurement, _meas_covariance)
{
    //std::cout << "New FeatureOdom2D: measurement " << _measurement.transpose() << std::endl << "covariance" << std::endl << _meas_covariance << std::endl;
}

FeatureOdom2D::~FeatureOdom2D()
{
    //
}

void FeatureOdom2D::findConstraints()
{

}
