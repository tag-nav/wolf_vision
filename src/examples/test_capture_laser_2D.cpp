/**
 * \file test_capture_laser_2D.cpp
 *
 *  Created on: 17/10/2014
 *     \author: acorominas
 */

//wolf
#include "vehicle_base.h"
#include "frame.h"
#include "capture_laser_2D.h"
#include "raw_laser_2D.h"
#include "feature_corner_2D.h"
#include "correspondence_base.h"
#include "state_pose.h"
#include "sensor_laser_2D.h"


//Eigen includes
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// std includes
#include <math.h>
#include <iostream>
#include <memory>

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
    double th;
    Vector3s v3s;
    Quaternions quat;
    VectorXs scan_data;
    shared_ptr<Frame> frame_shptr; 
    shared_ptr<StatePose> pose_shptr;    
    shared_ptr<CaptureLaser2D> capture_laser_shptr;        
    shared_ptr<RawLaser2D> raw_laser_shptr;//(new RawLaser2D(0.));//init with 0 timestamp
    shared_ptr<SensorLaser2D> laser_sensor_shptr;

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
    
    // 2. Initialize device params and pose wrt vehicle (once per execution.)
    StatePose sensor_pose;
    v3s << 1.,1.,1.;
    quat.coeffs() << 1.,0.,0.,0.;
    sensor_pose.p(v3s);
    sensor_pose.q(quat);
    laser_sensor_shptr.reset( new SensorLaser2D(sensor_pose, 180, M_PI, 0.1, 50.) );

    // 3. Initialize data. (Once per data arrival. When wrapping with ROS, this data will come from a ROS message)
    switch(scan_id)
    {
        case 1:
            /* This does not generate uniform angle measurements
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
             */
            
            //invented scan, some lines and 90deg corners
            scan_data.resize(180);
            for (unsigned int ii=0; ii<scan_data.size(); ii++) {
                th = ii*M_PI/180.0 -M_PI/2.0;
                yy = -10.0; xx = yy/tan(th);
                if (xx > 2.0) {
                    xx=2.0; yy=xx*tan(th);
                }
                if (yy > -8.0) {
                    yy = -8.0; xx = yy/tan(th);
                }
                if (xx > 4.0) {
                    xx=4.0; yy=xx*tan(th);
                }
                if (yy > -6.0) {
                    yy = -6.0; xx = yy/tan(th);
                }
                if (xx > 6.0) {
                    xx=6.0; yy=xx*tan(th);
                }
                if (yy > -4.0) {
                    yy = -4.0; xx = yy/tan(th);
                }
                if (((th>0) || (xx*xx+yy*yy > 50*50)) || (xx*xx+yy*yy < 0.1)) {
                    xx = 0; yy = 0;
                }
                scan_data(ii) = sqrt(xx*xx+yy*yy);
            }
            
            break;
        case 2:
            break; 
        default:
            break;
    }
    
    // 4. set timestamp and scan data to raw laser object. 
    raw_laser_shptr.reset( new RawLaser2D() );// time stamp of raw object is set to now
    raw_laser_shptr->setData(scan_data.size(), &scan_data(0)); 
    //raw_laser_shptr->print();

    // 5. Creates a new state and a new frame
    pose_shptr.reset( new StatePose() );   
    frame_shptr.reset( new Frame(nullptr, pose_shptr, raw_laser_shptr->timeStamp()) );

    // 6. Creates capture and add it to frame
    capture_laser_shptr.reset( new CaptureLaser2D(frame_shptr, laser_sensor_shptr) );
    capture_laser_shptr->setRaw(raw_laser_shptr);
    capture_laser_shptr->rawPtr()->print();
    frame_shptr->addCapture(capture_laser_shptr);

    // 7. Extract features
    capture_laser_shptr->processCapture();
    
    // 8. At this point, the tree should be built from frame up to features
    frame_shptr->print();
    
    
    // End of test
    cout << "\n==============================================================\n" << endl;

    return EXIT_SUCCESS;
}

