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

    sensors.push_back(SensorFactory::get()->createSensor(SEN_CAMERA, "left camera"));
    sensors.push_back(SensorFactory::get()->createSensor(SEN_CAMERA, "right camera"));
    sensors.push_back(SensorFactory::get()->createSensor(SEN_CAMERA, "rear camera"));
//    sensors.push_back(SensorFactory::get()->createSensor(SEN_IMU)); // this class not finished
    sensors.push_back(SensorFactory::get()->createSensor(SEN_ODOM_2D, "main odometer"));
    sensors.push_back(SensorFactory::get()->createSensor(SEN_ODOM_2D, "aux odometer"));

    for (auto sen : sensors){
        std::cout << "Sensor: " << sen->id() << " | name: " << sen->getName() << std::endl;
    }
    (*(sensors.rbegin()))->getName();

//    std::cout << "Asking for a CIRCLE which did not register to the factory..." << std::endl;
//    sensors.push_back(SensorFactory::get()->createSensor(SEN_GPS_RAW));
//    (*(sensors.rbegin()))->getName();


    return 0;
}

