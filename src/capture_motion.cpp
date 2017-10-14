/**
 * \file capture_motion2.cpp
 *
 *  Created on: Oct 14, 2017
 *      \author: jsola
 */

#include "capture_motion.h"

namespace wolf
{

CaptureMotion::CaptureMotion(const TimeStamp& _ts,
                             SensorBasePtr _sensor_ptr,
                             const Eigen::VectorXs& _data,
                             const Eigen::MatrixXs& _data_cov,
                             Size _delta_size, Size _delta_cov_size, Size _calib_size,
                             FrameBasePtr _origin_frame_ptr) :
        CaptureBase("MOTION", _ts, _sensor_ptr), data_(_data), data_cov_(_data_cov), calib_size_(_calib_size), buffer_(
                _data.size(), _delta_size, _delta_cov_size, _calib_size), origin_frame_ptr_(_origin_frame_ptr)
{
    //
}

CaptureMotion::~CaptureMotion()
{
    //
}

VectorXs CaptureMotion::getCalibration() const
{
    VectorXs calib(calib_size_);
    Size index = 0;
    for (Size i = 0; i < getStateBlockVec().size(); i++)
    {
        auto sb = getStateBlockPtr(i);
        if (sb && !sb->isFixed())
        {
            calib.segment(index, sb->getSize()) = sb->getState();
            index += sb->getSize();
        }
    }
    return calib;
}

Eigen::VectorXs CaptureMotion::getDelta()
{
    VectorXs calib_preint   = getCalibrationPreint();
    VectorXs calib          = getCalibration();
    VectorXs delta_preint   = getBuffer().get().back().delta_integr_;
    MatrixXs jac_calib      = getBuffer().get().back().jacobian_calib_;
    VectorXs delta_error    = jac_calib * (calib - calib_preint);
    VectorXs delta          = correctDelta(delta_preint, delta_error);
    return delta;
}

Eigen::VectorXs CaptureMotion::getDelta(const TimeStamp& _ts)
{
    VectorXs calib_preint   = getCalibrationPreint();
    VectorXs calib          = getCalibration();
    VectorXs delta_preint   = getBuffer().getMotion(_ts).delta_integr_;
    MatrixXs jac_calib      = getBuffer().getMotion(_ts).jacobian_calib_;
    VectorXs delta_error    = jac_calib * (calib - calib_preint);
    VectorXs delta          = correctDelta(delta_preint, delta_error);
    return delta;
}

}
