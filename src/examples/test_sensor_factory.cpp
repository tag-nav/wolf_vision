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

#include "../sensor_gps.h"
//#include "../processor_gps.h"

//#include "../sensor_laser_2D.h"

#include "../problem.h"


#include <iostream>
#include <iomanip>

// Put the registers down here just before the main():
namespace {
// Register factories at global scope!
const bool registered_sen_camera    = wolf::SensorFactory::get()->registerCreator("CAMERA",     wolf::SensorCamera::create);
const bool registered_sen_odom_2d   = wolf::SensorFactory::get()->registerCreator("ODOM 2D",    wolf::SensorOdom2D::create);
const bool registered_sen_gps_fix   = wolf::SensorFactory::get()->registerCreator("GPS FIX",    wolf::SensorGPSFix::create);
const bool registered_sen_gps       = wolf::SensorFactory::get()->registerCreator("GPS",        wolf::SensorGPS::create);
const bool registered_sen_imu       = wolf::SensorFactory::get()->registerCreator("IMU",        wolf::SensorIMU::create);
//const bool registered_laser         = wolf::SensorFactory::get()->registerCreator("LASER 2D",   wolf::SensorLaser2D::create);

const bool registered_prc_odom_3d   = wolf::ProcessorFactory::get()->registerCreator("ODOM 3D", wolf::ProcessorOdom3D::create);
const bool registered_prc_imu       = wolf::ProcessorFactory::get()->registerCreator("IMU",     wolf::ProcessorIMU::create);
const bool registered_prc_odom_2d   = wolf::ProcessorFactory::get()->registerCreator("ODOM 2D", wolf::ProcessorOdom2D::create);
//const bool registered_prc_laser_2d  = wolf::ProcessorFactory::get()->registerCreator("LASER 2D", wolf::ProcessorLaser2D::create);
//const bool registered_prc_gps       = wolf::ProcessorFactory::get()->registerCreator("GPS",     wolf::ProcessorGPS::create);
}

int main(void)
{
    using namespace wolf;
    using namespace std;

    cout << "\n================ Wolf Factories ================" << endl;

    cout << "If you look above, you see the registered methods.\nThere is one attempt per #include of the file, I ignore why!" << endl;

    Problem problem(FRM_PO_3D);

    cout << "\n================ Sensor Factory ================" << endl;

    // define some useful parameters
    Eigen::VectorXs pq_3d(7), po_2d(3), p_3d(3);
    IntrinsicsCamera intr_cam;
    IntrinsicsOdom2D intr_odom2d;
    IntrinsicsGPSFix intr_gps_fix;

    // Add sensors
    problem.addSensor("CAMERA",     "front left camera",      pq_3d,  &intr_cam);
    problem.addSensor("Camera",     "front right camera",     pq_3d,  &intr_cam);
    problem.addSensor("ODOM 2D",    "main odometer",    po_2d,  &intr_odom2d);
    problem.addSensor("GPS FIX",    "GPS fix",          p_3d,   &intr_gps_fix);
    problem.addSensor("CAMERA",     "rear camera",      pq_3d,  &intr_cam);
    problem.addSensor("IMU",        "inertial",         pq_3d,  &intr_cam);

    // Add this sensor and recover a pointer to it
    SensorBase* sen_ptr = problem.addSensor("ODOM 2D", "aux odometer", po_2d, &intr_odom2d);

    // print available sensors
    for (auto sen : *(problem.getHardwarePtr()->getSensorListPtr())){
        cout << "Sensor: " << setw(2) << left << sen->id()
                << " | type " << setw(2) << sen->typeId()
                << ": " << setw(8) << sen->getType()
                << " | name: " << sen->getName() << endl;
    }
    cout << sen_ptr->getName() << "\'s pointer: " << sen_ptr << " --------> All pointers are accessible if needed!" << endl;

    cout << "\n=============== Processor Factory ===============" << endl;

    // Add processors and bind them to sensors -- by sensor name!
    problem.addProcessor("ODOM 2D", "main odometry",    "main odometer",    nullptr);
    problem.addProcessor("ODOM 3D", "sec. odometry",    "aux odometer",     nullptr);
    problem.addProcessor("IMU",     "pre-integrated",   "inertial",         nullptr);

    // print installed processors
    for (auto sen : *(problem.getHardwarePtr()->getSensorListPtr()))
        for (auto prc : *(sen->getProcessorListPtr()))
            cout << "Processor: " << setw(2) << left  << prc->id()
            << " | type : " << setw(8) << prc->getType()
            << " | name: " << setw(15) << prc->getName()
            << " | bound to sensor: " << prc->getSensorPtr()->getName() << endl;


    return 0;
}

