/*
 * \file sensor_imu.h
 *
 *  Created on: 15/06/2014
 *      \author: jsola
 */

#ifndef SENSOR_IMU_H_
#define SENSOR_IMU_H_

#include "sensor_base.h"

/**
 * \brief Class for IMU sensor
 *
 * Inertial Measurement Unit (IMU) sensor
 */
class SensorIMU : public SensorBase
{
    public:
        /**
         * \brief Intrinsic parameters vector.
         *
         * These are, in order, the three accelerometer gains, and the three gyrometer gains.
         * All gains are with respect to SI units - m/s2 and rad/s.
         *
         * (NOTE: If static offsets are to be included, define a larger intrinsic with 12 parameters.)
         */
        static const unsigned int SIZE_PARAMS_ = 6; ///< 6 intrinsic parameters

        /**
         * \brief Constructor from pose
         * \param _sp the local pose with respect to the vehicle
         * All intrinsic gains initialized to 1.
         */
        SensorIMU(StatePose& _sp);

        /**
         * \brief Constructor from pose and gains vector
         * \param _sp the local pose with respect to the vehicle
         * \param _k a 6-vector with 3 acc gains and 3 gyro gains.
         */
        SensorIMU(StatePose& _sp, VectorXs& _k);

        virtual ~SensorIMU();
};
#endif /* SENSOR_IMU_H_ */
