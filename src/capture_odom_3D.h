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

// Declare the class
class CaptureOdom3D : public CaptureMotion2<odo3dDeltaType>
{
    public:
        CaptureOdom3D(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::Vector6s& _data);

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

#endif /* CAPTURE_ODOM_3D_H_ */
