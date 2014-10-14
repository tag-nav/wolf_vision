/*
 * imu_measurement.cpp
 *
 *  Created on: 01/06/2014
 *      \author: jsola
 */

#include <imu_measurement.h>

IMUMeasurement::IMUMeasurement(Vector3s& _am, Vector3s& _wm) :
        am_(_am), //
        wm_(_wm)//
{
}

IMUMeasurement::IMUMeasurement(VectorXs& _meas) :
    am_(_meas.segment(0,3)), //
    wm_(_meas.segment(3,3))//
{
    assert(_meas.size() == 6);
}

IMUMeasurement::IMUMeasurement() :
        am_(), //
        wm_()//
{
    //
}

IMUMeasurement::~IMUMeasurement()
{
    //
}

Vector3s& IMUMeasurement::am()
{
    return am_;
}

Vector3s& IMUMeasurement::wm()
{
    return wm_;
}
