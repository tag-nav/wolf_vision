/*
 * \file test_node.cpp
 *
 *  Created on: 01/07/2014
 *      \author: acorominas
 */

#include "node_linked.h"
#include "node_terminus.h"

using namespace std;

//id count init
unsigned int NodeBase::node_id_count_ = 0; 

//forward declarations
class FrameN;
class SensorDataN;

//class TrajectoryN
class TrajectoryN : public NodeLinked<NodeTerminus, FrameType, FrameN>
{
    protected:
        unsigned int length_; //just something to play
        
    public:
        TrajectoryN(const unsigned int _len) :
            NodeLinked<NodeTerminus, FrameType, FrameN>(TOP),
            length_(_len)
        {
            //
        };
        
        ~TrajectoryN()
        {
            
        };
};

//class FrameN
class FrameN : public NodeLinked<TrajectoryN, SensorType, SensorDataN>
{
    protected:
        double time_stamp_; //just something to play
        
    public:
        FrameN(double _ts) :
            NodeLinked<TrajectoryN, SensorType, SensorDataN>(MID),
            time_stamp_(_ts)
        {
            //
        };
        
        ~FrameN()
        {
            
        };
};

//class SensorDataN
class SensorDataN : public NodeLinked<FrameN, int, NodeTerminus>
{
    protected:
        unsigned int size_; //just something to play
        
    public:
        SensorDataN(double _sz) :
            NodeLinked<FrameN, int, NodeTerminus>(BOTTOM),
            size_(_sz)
        {
            //
        };
        
        ~SensorDataN()
        {
            
        };
};


int main()
{
    cout << endl << "Node class test" << endl;
    cout << "========================================================" << endl;

    cout << endl << "TEST 1. Constructors" << endl;
    shared_ptr<TrajectoryN> trajectory_(new TrajectoryN(2));
    shared_ptr<FrameN> frame_1_(new FrameN(1.011));
    shared_ptr<FrameN> frame_2_(new FrameN(2.022));
    shared_ptr<SensorDataN> sensor_data_cam_1_(new SensorDataN(640));
    shared_ptr<SensorDataN> sensor_data_laser_(new SensorDataN(180));
    shared_ptr<SensorDataN> sensor_data_cam_2_(new SensorDataN(480));
    shared_ptr<SensorDataN> sensor_data_radar_(new SensorDataN(90));
    trajectory_->print();
    cout << "========================================================" << endl;    

    cout << endl << "TEST 2. Build tree dependencies" << endl;
    frame_1_->addDownNode(CAMERA, sensor_data_cam_1_);
    sensor_data_cam_1_->linkToUpNode(frame_1_);
    
    frame_1_->addDownNode(LIDAR, sensor_data_laser_);
    sensor_data_laser_->linkToUpNode(frame_1_);
    
    frame_2_->addDownNode(CAMERA, sensor_data_cam_2_);
    sensor_data_cam_2_->linkToUpNode(frame_2_);
    
    trajectory_->addDownNode(KEY_FRAME, frame_1_);
    frame_1_->linkToUpNode(trajectory_);
    
    trajectory_->addDownNode(REGULAR_FRAME, frame_2_);
    frame_2_->linkToUpNode(trajectory_);
    trajectory_->print();
    cout << "========================================================" << endl;
    
    cout << endl << "TEST 3. Modify one of the nodes (add new node), once tree has been constructed" << endl;
    frame_2_->addDownNode(RADAR, sensor_data_radar_);    
    sensor_data_radar_->linkToUpNode(frame_2_);    
    
    trajectory_->print();
    cout << "========================================================" << endl;    
    
    cout << endl << "TEST 4. Remove nodes" << endl;
    //check if resetting ptr previously, effectively removes object when calling removeDownNode()
    unsigned int f1_id_ = frame_1_->getId();
    cout << __LINE__ << ": Calling reset()'s" << endl;
    frame_1_.reset();
    sensor_data_cam_1_.reset();
    sensor_data_laser_.reset();
    cout << __LINE__ << ": Calling removeDownNode()" << endl;
    trajectory_->removeDownNode(f1_id_);
    trajectory_->print();
    
    cout << "========================================================" << endl;    
    
    cout << endl << "End Node class test" << endl;
    return 0;
}

