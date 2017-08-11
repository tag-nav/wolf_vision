/*
 * feature_motion.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: jsola
 */

#include "feature_motion.h"

namespace wolf
{

FeatureMotion::FeatureMotion(const std::string& _type,
                             const Eigen::VectorXs& _measurement,
                             const Eigen::MatrixXs& _meas_covariance,
                             const Eigen::VectorXs& _calib_preint,
                             const Eigen::MatrixXs& _jacobian_calib) :
        FeatureBase(_type, _measurement, _meas_covariance),
        calib_preint_(_calib_preint),
        jacobian_calib_(_jacobian_calib)
{
    //
}

FeatureMotion::~FeatureMotion()
{
    //
}

} /* namespace wolf */
