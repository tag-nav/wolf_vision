/*
 * \file test_node.cpp
 *
 *  Created on: 01/07/2014
 *      \author: acorominas
 */

//wolf
#include "wolf.h"
#include "sensor_data_absolute_pose.h"

//namespaces
using namespace std;
using namespace Eigen;

//const 
const unsigned int NUM_FRAMES = 10;

//id count init
unsigned int NodeBase::node_id_count_ = 0; 

//main 
int main()
{
    //trajectory
    vector<shared_ptr<Frame> > trajectory_;
    
    //frames 
    shared_ptr<Frame> frame_ptr_;
    
    //state
    shared_ptr<StatePose> state_ptr_;
    
    //sensor callback 
    VectorXs data_(7);
    shared_ptr<RawAbsolutePose> raw_ptr_;
    shared_ptr<SensorDataAbsolutePose> sensor_data_ptr_;
    
    //start tests
    cout << endl << "SensorDataAbsolutePose class test" << endl;
    cout << "========================================================" << endl;

    cout << endl << "TEST 1. Constructors" << endl;
    for ( unsigned int ii=0; ii<NUM_FRAMES; ii++ )
    {
        //a prior state point
        data_ << ii*1.0, ii*2.0, ii*3.0, 1, 0, 0, 0;
        
        //creates a state
        state_ptr_ = make_shared<StatePose>(data_);
        
        //create a new frame
        frame_ptr_ = make_shared<Frame>(REGULAR_FRAME, ii/10., state_ptr_);
        
        //create raw sensor data
        data_ << ii*1.0+0.1, ii*2.0+0.1, ii*3.0+0.1, 1, 0, 0, 0;
        raw_ptr_ = make_shared<RawAbsolutePose>(ii/10. , data_);
//         sensor_data_ptr_ = make_shared<SensorDataAbsolutePose>();
        
        //push_back raw sensor data to the frame
//         frame_ptr_->addDownNode(ABSOLUTE_POSE, sensor_data_ptr_);
//         sensor_data_laser_->linkToUpNode(frame_1_);
        
        //push back the frame to the trajectory
        
        //reset local pointers
        frame_ptr_.reset();
        state_ptr_.reset();
    }
    
    cout << "========================================================" << endl;    

    return 0;
}

