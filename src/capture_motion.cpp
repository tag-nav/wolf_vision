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
                             Size _delta_size,
                             Size _delta_cov_size,
                             FrameBasePtr _origin_frame_ptr) :
        CaptureBase("MOTION", _ts, _sensor_ptr),
        data_(_data),
        data_cov_(_sensor_ptr ? _sensor_ptr->getNoiseCov() : Eigen::MatrixXs::Zero(_data.rows(), _data.rows())), // Someone should test this zero and do something smart accordingly
        buffer_(_data.size(), _delta_size, _delta_cov_size, computeCalibSize()),
        origin_frame_ptr_(_origin_frame_ptr)
{
    //
}

CaptureMotion::CaptureMotion(const TimeStamp& _ts,
                             SensorBasePtr _sensor_ptr,
                             const Eigen::VectorXs& _data,
                             const Eigen::MatrixXs& _data_cov,
                             Size _delta_size,
                             Size _delta_cov_size,
                             FrameBasePtr _origin_frame_ptr,
                             StateBlockPtr _p_ptr ,
                             StateBlockPtr _o_ptr ,
                             StateBlockPtr _intr_ptr ) :
                CaptureBase("MOTION", _ts, _sensor_ptr, _p_ptr, _o_ptr, _intr_ptr),
                data_(_data),
                data_cov_(_data_cov),
                buffer_(_data.size(), _delta_size, _delta_cov_size, computeCalibSize()),
                origin_frame_ptr_(_origin_frame_ptr)
{
    // TODO put something in the buffer to start!
}




CaptureMotion::~CaptureMotion()
{
    //
}


Eigen::VectorXs CaptureMotion::getDeltaCorrected(const VectorXs& _calib_current)
{
    VectorXs calib_preint   = getCalibrationPreint();
    VectorXs delta_preint   = getBuffer().get().back().delta_integr_;
    MatrixXs jac_calib      = getBuffer().get().back().jacobian_calib_;
    VectorXs delta_error    = jac_calib * (_calib_current - calib_preint);
    VectorXs delta          = correctDelta(delta_preint, delta_error);
    return delta;
}

Eigen::VectorXs CaptureMotion::getDeltaCorrected(const VectorXs& _calib_current, const TimeStamp& _ts)
{
    VectorXs calib_preint   = getCalibrationPreint();
    VectorXs delta_preint   = getBuffer().getMotion(_ts).delta_integr_;
    MatrixXs jac_calib      = getBuffer().getMotion(_ts).jacobian_calib_;
    VectorXs delta_error    = jac_calib * (_calib_current - calib_preint);
    VectorXs delta          = correctDelta(delta_preint, delta_error);
    return delta;
}

}
