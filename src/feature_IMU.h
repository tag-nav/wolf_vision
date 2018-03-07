#ifndef FEATURE_IMU_H_
#define FEATURE_IMU_H_

//Wolf includes
#include <capture_IMU.h>
#include "feature_base.h"
#include "wolf.h"

//std includes


namespace wolf {

//WOLF_PTR_TYPEDEFS(CaptureIMU);
WOLF_PTR_TYPEDEFS(FeatureIMU);

class FeatureIMU : public FeatureBase
{
    public:


        /** \brief Constructor from and measures
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         * \param _dD_db_jacobians Jacobians of preintegrated delta wrt IMU biases
         * \param acc_bias accelerometer bias of origin frame
         * \param gyro_bias gyroscope bias of origin frame
         * \param _cap_imu_ptr pointer to parent CaptureMotion
         */
        FeatureIMU(const Eigen::VectorXs& _delta_preintegrated,
                   const Eigen::MatrixXs& _delta_preintegrated_covariance,
                   const Eigen::Vector6s& _bias,
                   const Eigen::Matrix<wolf::Scalar,9,6>& _dD_db_jacobians,
                   CaptureMotionPtr _cap_imu_ptr = nullptr);

        /** \brief Constructor from capture pointer
         *
         * \param _cap_imu_ptr pointer to parent CaptureMotion
         */
        FeatureIMU(CaptureMotionPtr _cap_imu_ptr);

        virtual ~FeatureIMU();

        const Eigen::Vector3s&              getAccBiasPreint()  const;
        const Eigen::Vector3s&              getGyroBiasPreint() const;
        const Eigen::Matrix<Scalar, 9, 6>&  getJacobianBias()   const;

    private:

        // Used biases
        Eigen::Vector3s acc_bias_preint_;       ///< Acceleration bias used for delta preintegration
        Eigen::Vector3s gyro_bias_preint_;      ///< Gyrometer bias used for delta preintegration

        Eigen::Matrix<Scalar, 9, 6> jacobian_bias_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


inline const Eigen::Vector3s& FeatureIMU::getAccBiasPreint() const
{
    return acc_bias_preint_;
}

inline const Eigen::Vector3s& FeatureIMU::getGyroBiasPreint() const
{
    return gyro_bias_preint_;
}

inline const Eigen::Matrix<Scalar, 9, 6>& FeatureIMU::getJacobianBias() const
{
    return jacobian_bias_;
}

} // namespace wolf

#endif
