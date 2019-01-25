/*
 * capture_odom_2D.h
 *
 *  Created on: Oct 16, 2017
 *      Author: jsola
 */

#ifndef CAPTURE_ODOM_2D_H_
#define CAPTURE_ODOM_2D_H_

#include "capture_motion.h"

#include "rotations.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(CaptureOdom2D);

class CaptureOdom2D : public CaptureMotion
{
    public:
        CaptureOdom2D(const TimeStamp& _init_ts,
                      SensorBasePtr _sensor_ptr,
                      const Eigen::VectorXs& _data,
                      FrameBasePtr _origin_frame_ptr = nullptr);

        CaptureOdom2D(const TimeStamp& _init_ts,
                      SensorBasePtr _sensor_ptr,
                      const Eigen::VectorXs& _data,
                      const Eigen::MatrixXs& _data_cov,
                      FrameBasePtr _origin_frame_ptr = nullptr);

        virtual ~CaptureOdom2D();

        virtual VectorXs correctDelta(const VectorXs& _delta, const VectorXs& _delta_error) override;

};

inline Eigen::VectorXs CaptureOdom2D::correctDelta(const VectorXs& _delta, const VectorXs& _delta_error)
{
    Vector3s delta = _delta + _delta_error;
    delta(2) = pi2pi(delta(2));
    return delta;
}



} /* namespace wolf */

#endif /* CAPTURE_ODOM_2D_H_ */
