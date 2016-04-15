/**
 * \file capture_odom_2D2.h
 *
 *  Created on: 15/04/2016
 *      \author: jvallve
 */

#ifndef CAPTURE_ODOM_2D2_H_
#define CAPTURE_ODOM_2D2_H_

#include "capture_motion2.h"

#include "processor_odom_2D.h"

namespace wolf {

// Declare the class
class CaptureOdom2D2 : public CaptureMotion2
{
    public:
        CaptureOdom2D2(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::Vector2s& _data);

        // TODO This needs to go out!
    public:
        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now) const
        {
            return Eigen::VectorXs::Zero(3);
        }
};

inline CaptureOdom2D2::CaptureOdom2D2(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::Vector2s& _data) :
        CaptureMotion2(_ts, _sensor_ptr, _data)
{
    //
}

} // namespace wolf

#endif /* CAPTURE_ODOM_2D2_H_ */
