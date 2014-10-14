/*
 * \file test_node.cpp
 *
 *  Created on: 24/07/2014
 *      \author: acorominas
 */

//wolf
#include "node_linked.h"

//namespaces
using namespace std;

//id count init
unsigned int NodeBase::node_id_count_ = 0; 


//class TrajectoryN
class TrajectoryN : public NodeLinked
{
    protected:
        unsigned int length_; //just something to play
        
    public:
        TrajectoryN(const unsigned int _len) :
            NodeLinked(TOP),
            length_(_len)
        {
            //
        };
        
        ~TrajectoryN()
        {
            
        };
        
        virtual void printLabel(ostream & _ost = cout) const
        {
            _ost <<"TRAJECTORY";
        }        
};

//class FrameN
class FrameN : public NodeLinked
{
    protected:
        double time_stamp_; //just something to play
        
    public:
        FrameN(double _ts) :
            NodeLinked(MID),
            time_stamp_(_ts)
        {
            //
        };
        
        ~FrameN()
        {
            
        };
 
        virtual void printLabel(ostream & _ost = cout) const
        {
            _ost <<"FRAME";
        }

};

//class MeasurementN
class MeasurementN : public NodeLinked
{
    protected:
        unsigned int size_; //just something to play
        
    public:
        MeasurementN(const unsigned int _sz) :
            NodeLinked(BOTTOM),
            size_(_sz)
        {
            //
        };
        
        ~MeasurementN()
        {
            
        };
        
        virtual void printLabel(ostream & _ost = cout) const
        {
            _ost <<"MEASUREMENT";
        }
        
};


int main()
{
    cout << endl << "Node class test" << endl;
    cout << "========================================================" << endl;

    cout << endl << "TEST 1. Constructors" << endl;
    shared_ptr<TrajectoryN> trajectory_(new TrajectoryN(2));
    shared_ptr<FrameN> frame_1_(new FrameN(1.011));
    shared_ptr<FrameN> frame_2_(new FrameN(2.022));
    shared_ptr<MeasurementN> sensor_data_cam_1_(new MeasurementN(640));
    shared_ptr<MeasurementN> sensor_data_laser_(new MeasurementN(180));
    shared_ptr<MeasurementN> sensor_data_cam_2_(new MeasurementN(480));
    shared_ptr<MeasurementN> sensor_data_radar_(new MeasurementN(90));
    trajectory_->print();
    cout << "========================================================" << endl;    

    cout << endl << "TEST 2. Build tree dependencies" << endl;
    frame_1_->addDownNode(sensor_data_cam_1_);
    frame_1_->addDownNode(sensor_data_laser_);
    frame_2_->addDownNode(sensor_data_cam_2_);
    trajectory_->addDownNode(frame_1_);    
    trajectory_->addDownNode(frame_2_);
    trajectory_->print();
    cout << "========================================================" << endl;
    
    cout << endl << "TEST 3. Modify one of the nodes (add new node), once tree has been constructed" << endl;
    frame_2_->addDownNode(sensor_data_radar_);    
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
    
    cout << endl << "TEST 5. getTop()" << endl;
    shared_ptr<NodeLinked> sptr = sensor_data_radar_->getTop();
    cout << "TOP node: " << sptr->getId() << endl;
    cout << "========================================================" << endl;    
    
    
    cout << endl << "End Node class test" << endl;
    return 0;
}

