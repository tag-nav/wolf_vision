/**
 * \file feature_diff_drive.h
 *
 *  Created on: Oct 27, 2016
 *  \author: Jeremie Deray
 */

#ifndef _WOLF_FEATURE_DIFF_DRIVE_H_
#define _WOLF_FEATURE_DIFF_DRIVE_H_

//Wolf includes
#include "../feature_base.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(FeatureDiffDrive)

class FeatureDiffDrive : public FeatureBase
{
public:

  FeatureDiffDrive(const Eigen::VectorXs& _delta_preintegrated,
                   const Eigen::MatrixXs& _delta_preintegrated_covariance,
                   const Eigen::VectorXs& _diff_drive_factors,
                   const Eigen::MatrixXs& _jacobian_diff_drive_factors);

  virtual ~FeatureDiffDrive() = default;

  const Eigen::VectorXs& getJacobianFactor() const;

protected:

  Eigen::VectorXs diff_drive_factors_;
  Eigen::VectorXs jacobian_diff_drive_factors_;
};

} /* namespace wolf */

#endif /* _WOLF_FEATURE_DIFF_DRIVE_H_ */
