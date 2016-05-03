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

    std::cout << "================ Sensor Factory ================" << std::endl;

    Problem problem(FRM_PO_3D);

    Eigen::VectorXs extr_pq(7), extr_p(3);
    IntrinsicsCamera intr_cam;
    IntrinsicsOdom2D intr_odom2d;
    IntrinsicsBase intr;

    problem.addSensor("CAMERA",     "left camera",      extr_pq, &intr_cam);
    problem.addSensor("CAMERA",     "right camera",     extr_pq, &intr_cam);
    problem.addSensor("ODOM 2D",    "main odometer",    extr_pq, &intr_odom2d);
    problem.addSensor("GPS FIX",    "GPS fix",          extr_p,  &intr);
    problem.addSensor("CAMERA",     "rear camera",      extr_pq, &intr_cam);
    SensorBase* sen_ptr = problem.addSensor("ODOM 2D", "aux odometer", extr_pq, &intr_odom2d);

    for (auto sen : *(problem.getHardwarePtr()->getSensorListPtr())){
        std::cout << "Sensor: " << sen->id() << " | type " << sen->typeId() << ": " << sen->getType() << "\t| name: " << sen->getName() << std::endl;
    }

    std::cout << "aux odometer\'s pointer: " << sen_ptr << std::endl;

    return 0;
}

