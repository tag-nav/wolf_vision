/**
 * \file test_wolf_factories.cpp
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#include "../sensor_gps_fix.h"
#include "../sensor_camera.h"

#include "../sensor_odom_2D.h"
#include "../processor_odom_2D.h"
#include "../processor_odom_3D.h"

#include "../sensor_imu.h"
#include "../processor_imu.h"

#include "../sensor_gps.h"
#include "../processor_gps.h"

//#include "../sensor_laser_2D.h"

#include "../problem.h"


#include <iostream>
#include <iomanip>

int main(void)
{
    using namespace wolf;
    using namespace std;

    cout << "\n====== Registering creators in the Wolf Factories ======" << endl;

    cout << "If you look above, you see the registered creators.\n"
            "There is only one attempt per class, and it is successful!\n"
            "We do this by registering in the class\'s .cpp file.\n"
            "\n"
            "See [wolf]/src/examples/test_sensor_factory.cpp for the way to add sensors and processors to wolf::Problem." << endl;

    Problem problem(FRM_PO_3D);

    cout << "\n==================== Sensor Factory ====================" << endl;

    // define some useful parameters
    Eigen::VectorXs pq_3d(7), po_2d(3), p_3d(3);
    IntrinsicsCamera intr_cam;
    IntrinsicsOdom2D intr_odom2d;

    // Add sensors
    problem.installSensor("CAMERA",     "front left camera",    pq_3d,  &intr_cam);
    problem.installSensor("Camera",     "front right camera",   pq_3d,  &intr_cam);
    problem.installSensor("ODOM 2D",    "main odometer",        po_2d,  &intr_odom2d);
    problem.installSensor("GPS FIX",    "GPS fix",              p_3d);
    problem.installSensor("CAMERA",     "rear camera",          pq_3d,  &intr_cam);
    problem.installSensor("IMU",        "inertial",             pq_3d);
    problem.installSensor("GPS",        "GPS raw",              p_3d);

    // Add this sensor and recover a pointer to it
    SensorBase* sen_ptr = problem.installSensor("ODOM 2D", "aux odometer", po_2d, &intr_odom2d);

    // print available sensors
    for (auto sen : *(problem.getHardwarePtr()->getSensorListPtr())){
        cout << "Sensor " << setw(2) << left << sen->id()
                << " | type " << setw(2) << sen->typeId()
                << ": " << setw(8) << sen->getType()
                << " | name: " << sen->getName() << endl;
    }
    cout << sen_ptr->getName() << "\'s pointer: " << sen_ptr << " --------> All pointers are accessible if needed!" << endl;

    cout << "\n=================== Processor Factory ===================" << endl;

    // Add processors and bind them to sensors -- by sensor name!
    problem.installProcessor("ODOM 2D", "main odometry",    "main odometer");
    problem.installProcessor("ODOM 3D", "sec. odometry",    "aux odometer",     nullptr);
    problem.installProcessor("IMU",     "pre-integrated",   "inertial",         nullptr);
//    problem.createProcessor("GPS",     "GPS pseudoranges", "GPS raw",          nullptr);

    // print installed processors
    for (auto sen : *(problem.getHardwarePtr()->getSensorListPtr()))
        for (auto prc : *(sen->getProcessorListPtr()))
            cout << "Processor " << setw(2) << left  << prc->id()
            << " | type : " << setw(8) << prc->getType()
            << " | name: " << setw(17) << prc->getName()
            << " | bound to sensor " << setw(2) << prc->getSensorPtr()->id() << ": " << prc->getSensorPtr()->getName() << endl;


    return 0;
}

