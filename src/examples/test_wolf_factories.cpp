/**
 * \file test_wolf_factories.cpp
 *
 *  Created on: Apr 25, 2016
 *      \author: jsola
 */

#include "../sensor_gps_fix.h"
#include "../sensor_camera.h"
#include "../sensor_odom_2D.h"
#include "../sensor_imu.h"
#include "../sensor_gps.h"

#include "../processor_odom_2D.h"
#include "../processor_odom_3D.h"
#include "../processor_imu.h"
#include "../processor_gps.h"
#include "../processor_image.h"

#include "../problem.h"

#include "../factory.h"

#include <iostream>
#include <iomanip>
#include <cstdlib>


int main(void)
{
    using namespace wolf;
    using namespace std;


    /**=============================================================================================
     * Get wolf root directory from the environment variable WOLF_ROOT
     * To make this work, you need to set the environment variable WOLF_ROOT:
     *  - To run from terminal, edit your ~/.bashrc, or ~/.bash_profile and add this line:
     *        export WOLF_ROOT="/path/to/wolf"
     *  - To run from eclipse, open the 'run configuration' of this executable, tab 'Environment'
     *    and add variable WOLF_ROOT set to /path/to/wolf
     */
    char* w = std::getenv("WOLF_ROOT");
    if (w == NULL)
        throw std::runtime_error("Environment variable WOLF_ROOT not found");

    std::string WOLF_ROOT       = w;
    std::string WOLF_CONFIG     = WOLF_ROOT + "/src/examples";
    std::cout << "\nwolf directory for configuration files: " << WOLF_CONFIG << std::endl;
    //==============================================================================================

    cout << "\n====== Registering creators in the Wolf Factories =======" << endl;

    cout << "If you look above, you see the registered creators.\n"
            "There is only one attempt per class, and it is successful!\n"
            "We do this by registering in the class\'s .cpp file.\n"
            "\n"
            "- See \'" << WOLF_ROOT << "/src/examples/test_wolf_factories.cpp\'\n"
            "  for the way to install sensors and processors to wolf::Problem." << endl;

    // Start creating the problem

    Problem problem(FRM_PO_3D);

    // define some useful parameters
    Eigen::VectorXs pq_3d(7), po_2d(3), p_3d(3);
    IntrinsicsOdom2D intr_odom2d;

    cout << "\n================== Intrinsics Factory ===================" << endl;

    // Use params factory for camera intrinsics
    IntrinsicsBase* intr_cam_ptr = IntrinsicsFactory::get().create("CAMERA", WOLF_CONFIG + "/camera.yaml");
    ProcessorParamsBase* params_ptr = ProcessorParamsFactory::get().create("IMAGE", WOLF_CONFIG + "/processor_image_ORB.yaml");

    cout << "CAMERA with intrinsics      : " << ((IntrinsicsCamera*)intr_cam_ptr)->pinhole_model.transpose() << endl;
    cout << "Processor IMAGE image width : " << ((ProcessorImageParameters*)params_ptr)->image.width << endl;

    cout << "\n==================== Install Sensors ====================" << endl;

    // Install sensors
    problem.installSensor("CAMERA",     "front left camera",    pq_3d,  intr_cam_ptr);
    problem.installSensor("CAMERA",     "front right camera",   pq_3d,  WOLF_CONFIG + "/camera.yaml");
    problem.installSensor("ODOM 2D",    "main odometer",        po_2d,  &intr_odom2d);
    problem.installSensor("GPS FIX",    "GPS fix",              p_3d);
    problem.installSensor("IMU",        "inertial",             pq_3d);
    problem.installSensor("GPS",        "GPS raw",              p_3d);
    problem.installSensor("ODOM 2D",    "aux odometer",         po_2d,  &intr_odom2d);
    problem.installSensor("CAMERA", "rear camera", pq_3d, WOLF_ROOT + "/src/examples/camera.yaml");

    // print available sensors
    for (auto sen : *(problem.getHardwarePtr()->getSensorListPtr())){
        cout << "Sensor " << setw(2) << left << sen->id()
                << " | type " << setw(2) << sen->typeId()
                << ": " << setw(8) << sen->getType()
                << " | name: " << sen->getName() << endl;
    }

    cout << "\n=================== Install Processors ===================" << endl;

    // Install processors and bind them to sensors -- by sensor name!
    problem.installProcessor("ODOM 2D", "main odometry",    "main odometer");
    problem.installProcessor("ODOM 3D", "sec. odometry",    "aux odometer");
    problem.installProcessor("IMU",     "pre-integrated",   "inertial");
    problem.installProcessor("IMAGE",   "ORB",              "front left camera", WOLF_CONFIG + "/processor_image_ORB.yaml");
//    problem.createProcessor("GPS",     "GPS pseudoranges", "GPS raw");

    // print installed processors
    for (auto sen : *(problem.getHardwarePtr()->getSensorListPtr()))
        for (auto prc : *(sen->getProcessorListPtr()))
            cout << "Processor " << setw(2) << left  << prc->id()
            << " | type : " << setw(8) << prc->getType()
            << " | name: " << setw(17) << prc->getName()
            << " | bound to sensor " << setw(2) << prc->getSensorPtr()->id() << ": " << prc->getSensorPtr()->getName() << endl;


    return 0;
}

