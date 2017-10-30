#include "feature_diff_drive.h"

namespace wolf {

FeatureDiffDrive::FeatureDiffDrive(const Eigen::VectorXs& _delta_preintegrated,
                                   const Eigen::MatrixXs& _delta_preintegrated_covariance,
                                   const Eigen::VectorXs& _diff_drive_factors,
                                   const Eigen::MatrixXs& _jacobian_diff_drive_factors) :
  FeatureBase("DIFF DRIVE", _delta_preintegrated, _delta_preintegrated_covariance),
  diff_drive_factors_(_diff_drive_factors),
  jacobian_diff_drive_factors_(_jacobian_diff_drive_factors)
{
  //
}

const Eigen::VectorXs& FeatureDiffDrive::getJacobianFactor() const
{
  return jacobian_diff_drive_factors_;
}

} /* namespace wolf */
