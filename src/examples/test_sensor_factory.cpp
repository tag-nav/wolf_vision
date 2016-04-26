/**
 * \file test_sensor_factory.cpp
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#include "../sensor_camera.h"
#include "../sensor_odom_2d.h"
#include "../sensor_factory.h"
//#include "../sensor_imu.h" // this class not finished

#include <list>

int main(void)
{
    using namespace wolf;

    std::list<SensorBase*> sensors;

    std::cout << "========== Sensor Factory ==============" << std::endl;

    sensors.push_back(SensorFactory::get()->createSensor("CAMERA", "left camera"));
    sensors.push_back(SensorFactory::get()->createSensor("CAMERA", "right camera"));
//    sensors.push_back(SensorFactory::get()->createSensor("IMU", "MicroStrain IMU")); // this class not compiled
    sensors.push_back(SensorFactory::get()->createSensor("GPS_FIX", "GPS fix")); // this class not compiled
    sensors.push_back(SensorFactory::get()->createSensor("ODOM_2D", "main odometer"));
//    sensors.push_back(SensorFactory::get()->createSensor("LIDAR", "front laser")); // this class not compiled
    sensors.push_back(SensorFactory::get()->createSensor("CAMERA", "rear camera"));
//    sensors.push_back(SensorFactory::get()->createSensor("GPS_RAW", "raw GPS")); // this class not compiled
    sensors.push_back(SensorFactory::get()->createSensor("ODOM_2D", "aux odometer"));

    for (auto sen : sensors){
        std::cout << "Sensor: " << sen->id() << " | type: " << sen->type() << " | name: " << sen->getName() << std::endl;
    }
    (*(sensors.rbegin()))->getName();

//    std::cout << "Asking for a CIRCLE which did not register to the factory..." << std::endl;
//    sensors.push_back(SensorFactory::get()->createSensor("GPS_RAW));
//    (*(sensors.rbegin()))->getName();


    return 0;
}

