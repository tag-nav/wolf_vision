/*
 * feature_motion.h
 *
 *  Created on: Aug 11, 2017
 *      Author: jsola
 */

#ifndef FEATURE_MOTION_H_
#define FEATURE_MOTION_H_

#include "feature_base.h"
#include "capture_motion.h"

namespace wolf
{

class FeatureMotion : public FeatureBase
{
    public:
        FeatureMotion(const std::string& _type, const CaptureMotionPtr& _capture_motion);
        virtual ~FeatureMotion();

        Eigen::VectorXs getCalibrationPreint();
        Eigen::MatrixXs getJacobianCalibration();

        Eigen::VectorXs computeDeltaCorrection(const Eigen::VectorXs& _calib) const;
        Eigen::VectorXs getDeltaCorrected(const Eigen::VectorXs& _calib) const;

        virtual Eigen::VectorXs correctDelta(const Eigen::VectorXs& _delta, const Eigen::VectorXs& _delta_correction) const = 0;

    private:
        Eigen::VectorXs& calib_preint_;
        Eigen::MatrixXs& jacobian_calib_;
};

inline Eigen::VectorXs FeatureMotion::getCalibrationPreint()
{
    return calib_preint_;
}

inline Eigen::MatrixXs FeatureMotion::getJacobianCalibration()
{
    return jacobian_calib_;
}

inline Eigen::VectorXs FeatureMotion::computeDeltaCorrection(const Eigen::VectorXs& _calib) const
{
    return jacobian_calib_ * (_calib - calib_preint_);
}

inline Eigen::VectorXs FeatureMotion::getDeltaCorrected(const Eigen::VectorXs& _calib) const
{
    // delta_preint is stored in FeatureBase::measurement_;
    Eigen::VectorXs delta_step = computeDeltaCorrection(_calib);
    return correctDelta(measurement_, delta_step);
}

} /* namespace wolf */

#endif /* FEATURE_MOTION_H_ */
