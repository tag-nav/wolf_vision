/**
 * \file test_sensor_factory.cpp
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#include "../sensor_factory.h"
#include "../processor_factory.h"

#include "../sensor_gps_fix.h"
#include "../sensor_camera.h"

#include "../sensor_odom_2D.h"
#include "../processor_odom_2D.h"
#include "../processor_odom_3D.h"

#include "../sensor_imu.h"
#include "../processor_imu.h"

#include "../problem.h"

#include <iostream>
#include <iomanip>

int main(void)
{
    using namespace wolf;
    using namespace std;

    cout << "\n================ Wolf Factories ================" << endl;

    cout << "If you look above, you see the registered methods.\nThere is one attempt per #include of the file, I ignore why!" << endl;

    Problem problem(FRM_PO_3D);

    cout << "\n================ Sensor Factory ================" << endl;

    Eigen::VectorXs pq_3d(7), po_2d(3), p_3d(3);
    IntrinsicsCamera intr_cam;
    IntrinsicsOdom2D intr_odom2d;
    IntrinsicsGPSFix intr_gps_fix;

    problem.addSensor("CAMERA",     "front left camera",      pq_3d,  &intr_cam);
    problem.addSensor("Camera",     "front right camera",     pq_3d,  &intr_cam);
    problem.addSensor("ODOM 2D",    "main odometer",    po_2d,  &intr_odom2d);
    problem.addSensor("GPS FIX",    "GPS fix",          p_3d,   &intr_gps_fix);
    problem.addSensor("CAMERA",     "rear camera",      pq_3d,  &intr_cam);
    problem.addSensor("IMU",        "inertial",         pq_3d,  &intr_cam);

    SensorBase* sen_ptr = problem.addSensor("ODOM 2D", "aux odometer", po_2d, &intr_odom2d);

    for (auto sen : *(problem.getHardwarePtr()->getSensorListPtr())){
        cout << "Sensor: " << setw(2) << left << sen->id()
                << " | type " << setw(2) << sen->typeId()
                << ": " << setw(8) << sen->getType()
                << " | name: " << sen->getName() << endl;
    }

    cout << sen_ptr->getName() << "\'s pointer: " << sen_ptr << " --------> All pointers are accessible if needed!" << endl;

    cout << "\n=============== Processor Factory ===============" << endl;

    problem.addProcessor("ODOM 2D", "main odometry",    "main odometer",    nullptr);
    problem.addProcessor("ODOM 3D", "sec. odometry",    "aux odometer",     nullptr);
    problem.addProcessor("IMU",     "pre-integrated",   "inertial",         nullptr);

    for (auto sen : *(problem.getHardwarePtr()->getSensorListPtr()))
        for (auto prc : *(sen->getProcessorListPtr()))
            cout << "Processor: " << setw(2) << left  << prc->id()
            << " | type : " << setw(8) << prc->getType()
            << " | name: " << setw(15) << prc->getName()
            << " | bound to sensor: " << prc->getSensorPtr()->getName() << endl;


    return 0;
}

