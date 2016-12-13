#include "feature_imu.h"

namespace wolf {

FeatureIMU::FeatureIMU(const Eigen::VectorXs& _delta_preintegrated, const Eigen::MatrixXs& _delta_preintegrated_covariance) :
    FeatureBase("IMU", _delta_preintegrated, _delta_preintegrated_covariance),
    dp_preint_(_delta_preintegrated.head<3>()),dv_preint_(_delta_preintegrated.segment<3>(7)), dq_preint_(_delta_preintegrated.segment<4>(3))
{
    //std::cout << "New FeatureIMU: measurement " << _measurement.transpose() << std::endl << "covariance" << std::endl << _meas_covariance << std::endl;
}

FeatureIMU::~FeatureIMU()
{
    //
}

} // namespace wolf
