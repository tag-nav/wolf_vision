#include "feature_odom_2D.h"

namespace wolf {

FeatureOdom2D::FeatureOdom2D(unsigned int _dim_measurement) :
    FeatureBase(FEATURE_ODOM_2D, _dim_measurement)
{
    setType("ODOM 2D");
}

FeatureOdom2D::FeatureOdom2D(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase(FEATURE_ODOM_2D, _measurement, _meas_covariance)
{
    setType("ODOM 2D");
    //std::cout << "New FeatureOdom2D: measurement " << _measurement.transpose() << std::endl << "covariance" << std::endl << _meas_covariance << std::endl;
}

FeatureOdom2D::~FeatureOdom2D()
{
    //
}

void FeatureOdom2D::findConstraints()
{

}

} // namespace wolf
