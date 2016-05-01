/**
 * \file test_sensor_factory.cpp
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#include "../sensor_gps_fix.h"
#include "../sensor_camera.h"
#include "../sensor_odom_2D.h"
#include "../sensor_factory.h"
#include "../problem.h"
//#include "../sensor_imu.h" // this class not finished


int main(void)
{
    using namespace wolf;

    std::list<SensorBase*> sensors;

    std::cout << "================ Sensor Factory ================" << std::endl;

    Problem problem(FRM_PO_3D);

    problem.addSensor("CAMERA", "left camera", "");
    problem.addSensor("CAMERA", "right camera", "");
    problem.addSensor("ODOM 2D", "main odometer", "");
    problem.addSensor("GPS FIX", "GPS fix", "");
    problem.addSensor("CAMERA", "rear camera", "");
    SensorBase* sen_ptr = problem.addSensor("ODOM 2D", "aux odometer", "");

    for (auto sen : *(problem.getHardwarePtr()->getSensorListPtr())){
        std::cout << "Sensor: " << sen->id() << " | type " << sen->typeId() << ": " << sen->getType() << "\t| name: " << sen->getName() << std::endl;
    }

    std::cout << "aux odometer\'s pointer: " << sen_ptr << std::endl;

    return 0;
}

