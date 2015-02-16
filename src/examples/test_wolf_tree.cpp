// Testing a full wolf tree avoiding template classes for NodeLinked derived classes

//std includes
#include <iostream>
#include <memory>
#include <random>
#include <cmath>
#include <queue>

//ceres
#include "ceres/ceres.h"

//Wolf includes
#include "wolf.h"
#include "time_stamp.h"
#include "capture_base.h"
#include "sensor_base.h"


int main(int argc, char** argv) 
{    
    //wolf manager variables
    std::queue<FrameBase> trajectory_; //this will be owned by the wolf manager
    std::list<CaptureBaseShPtr> pending_captures_; //this will be owned by the wolf manager 
    std::list<CaptureBaseShPtr>::iterator pending_it_; //this will be owned by the wolf manager 
    Eigen::VectorXs sp(6);
    sp << 0.1,0.1,0.1,0,0,0;
    SensorBase sensor1(ABSOLUTE_POSE,sp); //just one sensor. This will be owned by the wolf manager
    sp << 0.2,0.2,0.2,0,0,0;
    SensorBase sensor2(ABSOLUTE_POSE,sp); //just another sensor. This will be owned by the wolf manager

    //ROS callbacks varaibles (will be get from message)
    TimeStamp ros_ts; //this plays the role of ros::Time, or msg->header.stamp
    Eigen::VectorXs sensor_reading(4); //this plays the role of the ROS message content (sensor reading). Reading of dim=4 (example)
    
    //Welcome message
    std::cout << std::endl << " ========= WOLF TREE test ===========" << std::endl << std::endl;

    //main loop
    for (unsigned int ii = 0; ii<10; ii++)
    {
        //1. a new sensor data arrives (this part will be placed on ROS callbacks)
        ros_ts.setToNow();
        sensor_reading << 1,2,3,4;
        std::shared_ptr<CaptureBase> capture( new CaptureBase(ros_ts.get(), &sensor1, sensor_reading) );
        pending_captures_.push_back(capture);
        
        //2. Process pending_captures_, deciding for each new capture wheter a Frame has to be created or they have to be linked to the last one
        for (pending_it_ = pending_captures_.begin(); pending_it_ != pending_captures_.end(); ++pending_it_)
        {
            capture->processCapture(); //This should create features    
            
        }
        
        //3. Stablish correspondences
        
        //4. Call ceres solver
        
        //5. publish results
        
    }
    
    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}
