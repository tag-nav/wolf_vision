/*
 * feature_motion.cpp
 *
 *  Created on: Aug 11, 2017
 *      Author: jsola
 */

#include "feature_motion.h"

namespace wolf
{

FeatureMotion::FeatureMotion(const std::string& _type, const CaptureMotionPtr& _capture_motion) :
        FeatureBase(_type, _capture_motion->getDeltaPreint(), _capture_motion->getDeltaPreintCov()),
        calib_preint_(_capture_motion->getCalibrationPreint()),
        jacobian_calib_(_capture_motion->getJacobianCalib())
{
    //
}

FeatureMotion::~FeatureMotion()
{
    //
}

} /* namespace wolf */
