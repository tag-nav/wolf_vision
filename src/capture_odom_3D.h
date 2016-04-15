/**
 * \file capture_odom_3D.h
 *
 *  Created on: 19/03/2016
 *      \author: jsola
 */

#ifndef CAPTURE_ODOM_3D_H_
#define CAPTURE_ODOM_3D_H_

#include "capture_motion2.h"

#include "processor_odom_3D.h"

namespace wolf {

/** \brief Capture for 3d odometry.
 *
 * This Capture stores motion data in the form of 3D odometry.
 *
 * The motion data_ is stored in the form of a 6-vector, containing the following components:
 *   - a 3d position increment in the local frame of the robot (dx, dy, dz)
 *   - a 3d orientation increment in the local frame of the robot (roll, pitch, yaw)
 *
 * All frames are assumed FLU (front, left, up).
 */
class CaptureOdom3D : public CaptureMotion2
{
    public:
        CaptureOdom3D(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::Vector6s& _data);
        virtual ~CaptureOdom3D();

        // TODO This needs to go out!
    public:
        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now) const
        {
            return Eigen::VectorXs::Zero(7);
        }
};

inline CaptureOdom3D::CaptureOdom3D(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::Vector6s& _data) :
        CaptureMotion2(_ts, _sensor_ptr, _data)
{
    //
}

inline CaptureOdom3D::~CaptureOdom3D()
{
}

} // namespace wolf

#endif /* CAPTURE_ODOM_3D_H_ */
