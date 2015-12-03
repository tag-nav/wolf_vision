//
// Created by ptirindelli on 3/12/15.
//

//std
#include <iostream>

//wolf
#include "sensor_gps.h"
#include <time_stamp.h>

using namespace std;

int main(int argc, char** argv)
{
    std::cout << std::endl << "GPS test" << std::endl;
    std::cout << "========================================================" << std::endl;

    //variable declarations and inits
    Eigen::VectorXs device_pose(6);
    device_pose << 0,0,0,0,0,0; //origin, no rotation

    //create the sensor with respect to the car
    SensorGPS device(new StateBlock(device_pose.head(3)), new StateBlock(device_pose.tail(3)));


    device.print(1, cout);

    cout << "the end\n";

    return 0;
}