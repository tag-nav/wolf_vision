#include "feature_imu.h"

namespace wolf {

FeatureIMU::FeatureIMU(unsigned int _dim_measurement) :
    FeatureBase(FEATURE_IMU, "IMU", _dim_measurement)
{
    //
}

FeatureIMU::FeatureIMU(const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_covariance) :
    FeatureBase(FEATURE_IMU, "IMU", _measurement, _meas_covariance)
{
    //std::cout << "New FeatureIMU: measurement " << _measurement.transpose() << std::endl << "covariance" << std::endl << _meas_covariance << std::endl;
}

FeatureIMU::~FeatureIMU()
{
    //
}

void FeatureIMU::findConstraints()
{

}

} // namespace wolf
