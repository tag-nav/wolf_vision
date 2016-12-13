#include "feature_imu.h"

namespace wolf {

FeatureIMU::FeatureIMU(const Eigen::VectorXs& _delta_preintegrated, const Eigen::MatrixXs& _delta_preintegrated_covariance) :
    FeatureBase("IMU", _delta_preintegrated, _delta_preintegrated_covariance),
    dp_preint_(_delta_preintegrated.head<3>()),dv_preint_(_delta_preintegrated.segment<3>(7)), dq_preint_(_delta_preintegrated.segment<4>(3))
    //acc_bias_preint_(_delta_preintegrated.segment<3>(10)), gyro_bias_preint_(_delta_preintegrated.tail<3>()) FIXME
{
    //std::cout << "New FeatureIMU: measurement " << _measurement.transpose() << std::endl << "covariance" << std::endl << _meas_covariance << std::endl;
}

FeatureIMU::FeatureIMU(const Eigen::VectorXs& _delta_preintegrated, const Eigen::MatrixXs& _delta_preintegrated_covariance, const Eigen::Matrix<wolf::Scalar,9,6>& _dD_db_jacobians) :
    FeatureBase("IMU", _delta_preintegrated, _delta_preintegrated_covariance),
    dp_preint_(_delta_preintegrated.head<3>()),dv_preint_(_delta_preintegrated.segment<3>(7)), dq_preint_(_delta_preintegrated.segment<4>(3)),
    acc_bias_preint_(_delta_preintegrated.segment<3>(10)), gyro_bias_preint_(_delta_preintegrated.tail<3>()),
    dDp_dab_(_dD_db_jacobians.block(3,3,0,0)), dDv_dab_(_dD_db_jacobians.block(3,3,6,0)),
    dDp_dwb_(_dD_db_jacobians.block(3,3,0,3)), dDv_dwb_(_dD_db_jacobians.block(3,3,6,3)), dDq_dwb_(_dD_db_jacobians.block(3,3,3,3))
{
    //std::cout << "New FeatureIMU: measurement " << _measurement.transpose() << std::endl << "covariance" << std::endl << _meas_covariance << std::endl;
}

FeatureIMU::~FeatureIMU()
{
    //
}

} // namespace wolf
