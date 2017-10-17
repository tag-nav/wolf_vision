/*
 * capture_odom_3D.h
 *
 *  Created on: Oct 16, 2017
 *      Author: jsola
 */

#ifndef CAPTURE_ODOM_3D_H_
#define CAPTURE_ODOM_3D_H_

#include "capture_motion.h"

#include "rotations.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(CaptureOdom3D);

class CaptureOdom3D : public CaptureMotion
{
    public:
        CaptureOdom3D(const TimeStamp& _init_ts,
                      SensorBasePtr _sensor_ptr,
                      const Eigen::Vector3s& _data,
                      FrameBasePtr _origin_frame_ptr = nullptr);

        CaptureOdom3D(const TimeStamp& _init_ts,
                      SensorBasePtr _sensor_ptr,
                      const Eigen::Vector3s& _data,
                      const Eigen::MatrixXs& _data_cov,
                      FrameBasePtr _origin_frame_ptr = nullptr);

        virtual ~CaptureOdom3D();

        virtual VectorXs correctDelta(const VectorXs& _delta, const VectorXs& _delta_error) override;

};

} /* namespace wolf */


#endif /* CAPTURE_ODOM_3D_H_ */
