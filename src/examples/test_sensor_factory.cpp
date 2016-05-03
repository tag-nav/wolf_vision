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

    Eigen::VectorXs pq_3d(7), po_2d(3), p_3d(3);
    IntrinsicsCamera intr_cam;
    IntrinsicsOdom2D intr_odom2d;
    IntrinsicsGPSFix intr_gps_fix;

    problem.addSensor("CAMERA",     "left camera",      pq_3d,  &intr_cam);
    problem.addSensor("CAMERA",     "right camera",     pq_3d,  &intr_cam);
    problem.addSensor("ODOM 2D",    "main odometer",    po_2d,  &intr_odom2d);
    problem.addSensor("GPS FIX",    "GPS fix",          p_3d,   &intr_gps_fix);
    problem.addSensor("CAMERA",     "rear camera",      pq_3d,  &intr_cam);

    SensorBase* sen_ptr = problem.addSensor("ODOM 2D", "aux odometer", po_2d, &intr_odom2d);

    for (auto sen : *(problem.getHardwarePtr()->getSensorListPtr())){
        std::cout << "Sensor: " << sen->id() << " | type " << sen->typeId() << ": " << sen->getType() << "\t| name: " << sen->getName() << std::endl;
    }

    std::cout << "aux odometer\'s pointer: " << sen_ptr << std::endl;

    return 0;
}

