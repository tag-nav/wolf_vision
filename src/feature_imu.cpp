#include "feature_imu.h"

namespace wolf {

FeatureIMU::FeatureIMU(const Eigen::VectorXs& _delta_preintegrated, const Eigen::MatrixXs& _delta_preintegrated_covariance) :
    FeatureBase("IMU", _delta_preintegrated, _delta_preintegrated_covariance),
    dp_preint_(_delta_preintegrated.head<3>()),dv_preint_(_delta_preintegrated.tail<3>()), dq_preint_(_delta_preintegrated.segment<4>(3))
{
    //std::cout << "New FeatureIMU: measurement " << _measurement.transpose() << std::endl << "covariance" << std::endl << _meas_covariance << std::endl;
}

FeatureIMU::FeatureIMU(const Eigen::VectorXs& _delta_preintegrated, const Eigen::MatrixXs& _delta_preintegrated_covariance, const Eigen::Vector3s& _acc_bias, const Eigen::Vector3s& _gyro_bias, const Eigen::Matrix<wolf::Scalar,9,6>& _dD_db_jacobians) :
    FeatureBase("IMU", _delta_preintegrated, _delta_preintegrated_covariance),
    dp_preint_(_delta_preintegrated.head<3>()),dv_preint_(_delta_preintegrated.tail<3>()), dq_preint_(_delta_preintegrated.segment<4>(3)),
    acc_bias_preint_(_acc_bias), gyro_bias_preint_(_gyro_bias),
    dDp_dab_(_dD_db_jacobians.block(0,0,3,3)), dDv_dab_(_dD_db_jacobians.block(6,0,3,3)),
    dDp_dwb_(_dD_db_jacobians.block(0,3,3,3)), dDv_dwb_(_dD_db_jacobians.block(6,3,3,3)), dDq_dwb_(_dD_db_jacobians.block(3,3,3,3))
{
    //std::cout << "New FeatureIMU: measurement " << _measurement.transpose() << std::endl << "covariance" << std::endl << _meas_covariance << std::endl;
}

FeatureIMU::FeatureIMU(const Eigen::VectorXs& _delta_preintegrated, const Eigen::MatrixXs& _delta_preintegrated_covariance, const wolf::CaptureIMUPtr _cap_imu_ptr, const Eigen::Matrix<wolf::Scalar,9,6>& _dD_db_jacobians): 
FeatureBase("IMU", _delta_preintegrated, _delta_preintegrated_covariance)
{
    //TODO : SIZE ASSERTIONS : make sure _delta_preintegrated and _delta_preintegrated_covariance sizes are correct !

    this->setCapturePtr(_cap_imu_ptr);
    //TODO : we should make sure here that there is a frame in the capture
    dp_preint_ = _delta_preintegrated.head<3>();
    dv_preint_ = _delta_preintegrated.tail<3>();
    dq_preint_ = _delta_preintegrated.segment<4>(3);

    acc_bias_preint_ = std::static_pointer_cast<wolf::FrameIMU>(_cap_imu_ptr->getFramePtr())->getAccBiasPtr()->getState();
    gyro_bias_preint_ = std::static_pointer_cast<wolf::FrameIMU>(_cap_imu_ptr->getFramePtr())->getGyroBiasPtr()->getState();

    dDp_dab_ = _dD_db_jacobians.block(0,0,3,3);
    dDv_dab_ = _dD_db_jacobians.block(6,0,3,3);
    dDp_dwb_ = _dD_db_jacobians.block(0,3,3,3);
    dDv_dwb_ = _dD_db_jacobians.block(6,3,3,3); 
    dDq_dwb_ = _dD_db_jacobians.block(3,3,3,3);
}

FeatureIMU::~FeatureIMU()
{
    //
}

} // namespace wolf
