/*
 * imu_measurement.h
 *
 *  Created on: 01/06/2014
 *      \author: jsola
 */

#ifndef IMU_MEASUREMENT_H_
#define IMU_MEASUREMENT_H_

#include <eigen3/Eigen/Dense>

using namespace Eigen;

/**
 * \brief Class for IMU measurements
 */
class IMUMeasurement
{
    public:
        IMUMeasurement();
        IMUMeasurement(Vector3s& _am, Vector3s& _wm);
        IMUMeasurement(VectorXs& _meas);
        virtual ~IMUMeasurement();

        Vector3s& am();
        Vector3s& wm();

    private:
        Vector3s am_, wm_;

};

#endif /* IMU_MEASUREMENT_H_ */
