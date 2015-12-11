//
// Created by ptirindelli on 4/12/15.
//

#ifndef CAPTURE_GPS_H_
#define CAPTURE_GPS_H_


// Wolf includes
#include "capture_base.h"
#include "feature_gps_pseudorange.h"
#include "constraint_gps_pseudorange.h"


//TODO indentation: add a tab at everything inside the class: protected and public must have 1 tab
class CaptureGPS : public CaptureBase
{
public:
    /* TODO rawData data type
     * i will not have a vector of float as raw data,
     * but i'll have to create a virtual class rawBase and then
     * a rawGPS that implement it
     *
     * then use this class as the type of the third argument,
     * and add a constructor in CaptureBase that work with
     * this rawGPS
     */




    /* TODO position/orientation of antenna
     * CaptureBase has also
     * Eigen::Vector3s sensor_pose_global_; ///< Sensor pose in world frame: composition of the frame pose and the sensor pose. TODO: use state units
     * Eigen::Vector3s inverse_sensor_pose_; ///< World pose in the sensor frame: inverse of the global_pose_. TODO: use state units
     *
     * they are reachable from this class, because private,
     * but they don't have getter/setter
     */

    CaptureGPS(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _raw_data);


    virtual ~CaptureGPS();

    void processCapture();

    virtual Eigen::VectorXs computePrior(const TimeStamp &_now) const;

protected:
    Eigen::VectorXs raw_data_;
};


#endif //CAPTURE_GPS_H_
