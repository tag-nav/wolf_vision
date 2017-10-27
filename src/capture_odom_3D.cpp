/*
 * capture_odom_3D.cpp
 *
 *  Created on: Oct 16, 2017
 *      Author: jsola
 */

#include "capture_odom_3D.h"

namespace wolf
{

CaptureOdom3D::CaptureOdom3D(const TimeStamp& _init_ts,
                             SensorBasePtr _sensor_ptr,
                             const Eigen::Vector6s& _data,
                             FrameBasePtr _origin_frame_ptr):
        CaptureMotion(_init_ts, _sensor_ptr, _data, 7, 6, _origin_frame_ptr)
{
    setType("ODOM 3D");
}

CaptureOdom3D::CaptureOdom3D(const TimeStamp& _init_ts,
                             SensorBasePtr _sensor_ptr,
                             const Eigen::Vector6s& _data,
                             const Eigen::MatrixXs& _data_cov,
                             FrameBasePtr _origin_frame_ptr):
        CaptureMotion(_init_ts, _sensor_ptr, _data, _data_cov, 7, 6, _origin_frame_ptr)
{
    setType("ODOM 3D");
}

CaptureOdom3D::~CaptureOdom3D()
{
    //
}

Eigen::VectorXs CaptureOdom3D::correctDelta(const VectorXs& _delta, const VectorXs& _delta_error)
{
    VectorXs delta(7);
    delta.head(3) = _delta.head(3) + _delta_error.head(3);
    delta.tail(4) = (Quaternions(_delta.head(4).data()) * exp_q(_delta_error.tail(3))).coeffs();
    return delta;
}

} /* namespace wolf */



