#include "feature_odom_2D.h"

FeatureOdom2D::FeatureOdom2D(unsigned int _dim_measurement) :
    FeatureBase(_dim_measurement)
{
    //
}

FeatureOdom2D::FeatureOdom2D(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase(_measurement, _meas_covariance)
{
	//
}

FeatureOdom2D::~FeatureOdom2D()
{
    //
}

void FeatureOdom2D::findConstraints()
{

}
