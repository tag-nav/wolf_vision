#ifndef FEATURE_IMU_H_
#define FEATURE_IMU_H_

//Wolf includes
#include "feature_base.h"
#include "capture_imu.h"
#include "frame_imu.h"
#include "wolf.h"

//std includes


namespace wolf {

//WOLF_PTR_TYPEDEFS(CaptureIMU);
WOLF_PTR_TYPEDEFS(FeatureIMU);

//class FeatureIMU
class FeatureIMU : public FeatureBase
{
    public:

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         *
         */
        FeatureIMU(const Eigen::VectorXs& _delta_preintegrated, const Eigen::MatrixXs& _delta_preintegrated_covariance);

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         * \param _dD_db_jacobians Jacobians of preintegrated delta wrt IMU biases
         * \param acc_bias accelerometer bias of origin frame
         * \param gyro_bias gyroscope bias of origin frame
         *
         */
        FeatureIMU(const Eigen::VectorXs& _delta_preintegrated,
                   const Eigen::MatrixXs& _delta_preintegrated_covariance,
                   const Eigen::Vector3s& _acc_bias,
                   const Eigen::Vector3s& _gyro_bias,
                   const Eigen::Matrix<wolf::Scalar,9,6>& _dD_db_jacobians);

        /** \brief Constructor from capture pointer and measure
         *
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         * \param _dD_db_jacobians Jacobians of preintegrated delta wrt IMU biases
         * \param _cap_imu_ptr is the CaptureIMUPtr pointing to desired capture containing the origin frame
         *
         */
        FeatureIMU(const Eigen::VectorXs& _delta_preintegrated,
                   const Eigen::MatrixXs& _delta_preintegrated_covariance,
                   const wolf::CaptureIMUPtr _cap_imu_ptr,
                   const Eigen::Matrix<wolf::Scalar,9,6>& _dD_db_jacobians);

        virtual ~FeatureIMU();

        const Eigen::Vector3s&              getDpPreint()       const;
        const Eigen::Quaternions&           getDqPreint()       const;
        const Eigen::Vector3s&              getDvPreint()       const;
        const Eigen::Vector3s&              getAccBiasPreint()  const;
        const Eigen::Vector3s&              getGyroBiasPreint() const;
        const Eigen::Matrix<Scalar, 9, 6>&  getJacobianBias()   const;

    private:
        /// Preintegrated delta
        Eigen::Vector3s dp_preint_;
        Eigen::Vector3s dv_preint_;
        Eigen::Quaternions dq_preint_;

        // Used biases
        Eigen::Vector3s acc_bias_preint_;       ///< Acceleration bias used for delta preintegration
        Eigen::Vector3s gyro_bias_preint_;      ///< Gyrometer bias used for delta preintegration

        Eigen::Matrix<Scalar, 9, 6> jacobian_bias_;

};

inline const Eigen::Vector3s& FeatureIMU::getDpPreint() const
{
    return dp_preint_;
}

inline const Eigen::Quaternions& FeatureIMU::getDqPreint() const
{
    return dq_preint_;
}

inline const Eigen::Vector3s& FeatureIMU::getDvPreint() const
{
    return dv_preint_;
}

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
