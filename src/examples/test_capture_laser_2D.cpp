//wolf
#include "capture_laser_2D.h"

//main
int main(int argc, char *argv[])
{
    std::cout << std::endl << "CaptureLaser2D class test" << std::endl;
    std::cout << "========================================================" << std::endl;
    
    //scan ranges
    Eigen::VectorXs ranges;
    ranges << 1,2,3;
    
    //variable declarations and inits
    Eigen::VectorXs device_pose;
    device_pose << 0,0,0,0,0,0; //origin, no rotation
    TimeStamp time_stamp;
    time_stamp.setToNow();
    
    //Device and Capture declaration
    SensorLaser2D device(device_pose, 180, M_PI, 0.2, 30.0, 0.01);
    CaptureLaser2D capture(time_stamp, &device, ranges);
    
    //do things with the measurements
    

    std::cout << "========================================================" << std::endl;            
    std::cout << std::endl << "End CaptureLaser2D class test" << std::endl;
    return 0;
}

