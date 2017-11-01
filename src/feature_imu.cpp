#include "feature_imu.h"

namespace wolf {

FeatureIMU::FeatureIMU(const Eigen::VectorXs& _delta_preintegrated,
                       const Eigen::MatrixXs& _delta_preintegrated_covariance,
                       const Eigen::Vector6s& _bias,
                       const Eigen::Matrix<wolf::Scalar,9,6>& _dD_db_jacobians,
                       CaptureMotionPtr _cap_imu_ptr) :
    FeatureBase("IMU", _delta_preintegrated, _delta_preintegrated_covariance),
    acc_bias_preint_(_bias.head<3>()),
    gyro_bias_preint_(_bias.tail<3>()),
    jacobian_bias_(_dD_db_jacobians)
{
    if (_cap_imu_ptr)
        this->setCapturePtr(_cap_imu_ptr);
    WOLF_TRACE("jac bias: ", jacobian_bias_.row(0));
}

FeatureIMU::FeatureIMU(CaptureMotionPtr _cap_imu_ptr):
        FeatureBase("IMU", _cap_imu_ptr->getDeltaPreint(), _cap_imu_ptr->getDeltaPreintCov()),
        acc_bias_preint_ (_cap_imu_ptr->getCalibrationPreint().head<3>()),
        gyro_bias_preint_(_cap_imu_ptr->getCalibrationPreint().tail<3>()),
        jacobian_bias_(_cap_imu_ptr->getJacobianCalib())
{
    WOLF_TRACE("jac bias: ", jacobian_bias_.row(0));

    this->setCapturePtr(_cap_imu_ptr);
}

FeatureIMU::~FeatureIMU()
{
    //
}

} // namespace wolf
