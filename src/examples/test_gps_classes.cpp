//std
#include <iostream>

//wolf
#include "sensor_gps.h"
#include "capture_gps.h"


using namespace std;

int main(int argc, char** argv)
{
    std::cout << std::endl << "GPS test" << std::endl;
    std::cout << "========================================================" << std::endl;

    //variable declarations and inits
    Eigen::VectorXs device_pose(6);
    device_pose << 0,0,0,0,0,0; //origin, no rotation
    //create the device with respect to the car

    SensorGPS device(new StateBlock(device_pose.head(3)), new StateBlock(device_pose.tail(3)));
    device.print(1, cout);

    cout << "----------------\n";

    //create a capture objects
    TimeStamp time_stamp;
    time_stamp.setToNow();

    Eigen::VectorXs raw_data(4);
    raw_data << 42, 43, 44, 45;

    CaptureGPS capture(time_stamp, &device, raw_data);
    capture.processCapture();


    return 0;
}
