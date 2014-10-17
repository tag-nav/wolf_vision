/**
 * \file test_capture_laser_2D.cpp
 *
 *  Created on: 17/10/2014
 *     \author: acorominas
 */

//wolf
#include "wolf.h"
// #include "frame.h"
// #include "capture_laser_2D.h"
// #include "raw_laser_2D.h"
// #include "feature_corner_2D.h"
// #include "state_pose.h"

// std includes
#include <math.h>
#include <iostream>

//namespaces
using namespace std;
using namespace Eigen;

//const 
const unsigned int LASER_STEP_SIZE = 20;

int main(int argc, char *argv[])
{
    unsigned int scan_id;
    double xx;
    double yy;
    

    // 1.  Parse input parameters
    cout << "\nTest for Laser Scan Capture - test_capture_laser_2D.cpp";
    cout << "\n=======================================================\n" << endl;
    if(argc < 2) 
    {
        cout << "Usage: test_capture_laser_2D [scan id] : Checks class against the given hardcoded scan id" << endl;
        return EXIT_SUCCESS;
    }
    else
    {
        scan_id = strtoul(argv[1], NULL, 10);
    }
    cout << "Testing with scan id: " << scan_id << endl << endl;

    // 2. Initialize data. When wrapping with ROS, this data will come from a ROS message
    VectorXs scan_data;
    switch(scan_id)
    {
        case 1: 
            //invented scan, some lines and 90deg corners
            scan_data.resize(LASER_STEP_SIZE*9); 
            xx = 0; yy = -10;
            for (unsigned int ii=0; ii<scan_data.size(); ii++)
            {
                if (ii<LASER_STEP_SIZE) xx += 0.1;
                if ( (ii>LASER_STEP_SIZE) && (ii<=LASER_STEP_SIZE*2) ) yy += 0.1;
                if ( (ii>LASER_STEP_SIZE*2) && (ii<=LASER_STEP_SIZE*3) ) xx += 0.1;
                if ( (ii>LASER_STEP_SIZE*3) && (ii<=LASER_STEP_SIZE*4) ) yy += 0.1;
                if ( (ii>LASER_STEP_SIZE*4) && (ii<=LASER_STEP_SIZE*5) ) xx += 0.1;
                if ( (ii>LASER_STEP_SIZE*5) && (ii<=LASER_STEP_SIZE*6) ) yy += 0.1;
                if ( (ii>LASER_STEP_SIZE*6) && (ii<=LASER_STEP_SIZE*7) ) xx -= 0.05;
                if ( (ii>LASER_STEP_SIZE*7) && (ii<=LASER_STEP_SIZE*8) ) yy += 0.1;
                if ( (ii>LASER_STEP_SIZE*8) && (ii<=LASER_STEP_SIZE*9) ) xx -= 0.05;
                scan_data(ii) = sqrt(xx*xx+yy*yy);                    
            }
            break;
        case 2:
            break; 
        default:
            break;
    }
    
    // 3. set scan data to raw object. When wrapping with ROS, this step will be done by the callback
    
    

    return EXIT_SUCCESS;
}

