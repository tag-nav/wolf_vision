/*
 * capture_odom_2D.cpp
 *
 *  Created on: Oct 16, 2017
 *      Author: jsola
 */

#include "base/capture/capture_odom_2D.h"

namespace wolf
{

CaptureOdom2D::CaptureOdom2D(const TimeStamp& _init_ts,
                             SensorBasePtr _sensor_ptr,
                             const Eigen::VectorXs& _data,
                             FrameBasePtr _origin_frame_ptr):
        CaptureMotion("ODOM 2D", _init_ts, _sensor_ptr, _data, 3, 3, _origin_frame_ptr)
{
    //
}

CaptureOdom2D::CaptureOdom2D(const TimeStamp& _init_ts,
                             SensorBasePtr _sensor_ptr,
                             const Eigen::VectorXs& _data,
                             const Eigen::MatrixXs& _data_cov,
                             FrameBasePtr _origin_frame_ptr):
        CaptureMotion("ODOM 2D", _init_ts, _sensor_ptr, _data, _data_cov, 3, 3, _origin_frame_ptr)
{
    //
}

CaptureOdom2D::~CaptureOdom2D()
{
    //
}

} /* namespace wolf */
