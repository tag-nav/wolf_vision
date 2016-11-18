#ifndef FEATURE_IMU_H_
#define FEATURE_IMU_H_

//Wolf includes
#include "feature_base.h"

//std includes


namespace wolf {
    
//forward declaration to typedef class pointers
class FeatureIMU;
typedef std::shared_ptr<FeatureIMU> FeatureIMUPtr;
typedef std::shared_ptr<const FeatureIMU> FeatureIMUConstPtr;
typedef std::weak_ptr<FeatureIMU> FeatureIMUWPtr;

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

        virtual ~FeatureIMU();

    public: // TODO eventually produce getters for these and then go private
        /// Preintegrated delta
        Eigen::Vector3s dp_preint_;
        Eigen::Vector3s dv_preint_;
        Eigen::Quaternions dq_preint_;

        // Used biases
        Eigen::Vector3s acc_bias_preint_;       ///< Accleration bias used for delta preintegration
        Eigen::Vector3s gyro_bias_preint_;      ///< Gyrometer bias used for delta preintegration

        // Jacobians of preintegrated deltas wrt biases
        Eigen::Matrix3s dDp_dab_;
        Eigen::Matrix3s dDv_dab_;
        Eigen::Matrix3s dDp_dwb_;
        Eigen::Matrix3s dDv_dwb_;
        Eigen::Matrix3s dDq_dwb_;

};

} // namespace wolf

#endif
