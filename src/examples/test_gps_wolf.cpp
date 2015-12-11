// Testing create and delete full wolf tree with GPS captures

//std includes
#include <iostream>
#include <memory>
#include <random>
#include <cmath>
#include <queue>

//Wolf includes
#include "wolf_manager.h"

#include "sensor_gps.h" //TODO to bu putted inside wolg_manager.h?


using namespace std;

int main(int argc, char** argv)
{
    //Welcome message
    cout << endl << " ========= WOLF TREE test ===========" << endl << endl;

    SensorGPS* gps_sensor_ptr_ = new SensorGPS(new StateBlock(Eigen::Vector3s::Zero()),
                                               new StateBlock(Eigen::Vector4s::Zero()));


    WolfManager* wolf_manager_ = new WolfManager(PO_3D,                             //frame structure
                                                 gps_sensor_ptr_,                   //gps raw sensor
                                                 Eigen::Vector3s::Zero(),           //prior
                                                 Eigen::Matrix3s::Identity()*0.01,  //prior cov
                                                 5,                                 //window size
                                                 1);                                //time for new keyframe
    //TODO qui sopra ho errori

//    wolf_manager_->addSensor(gps_sensor_ptr_);
//    wolf_manager_->getProblemPtr()->print();

//
//    //FeatureBase* feature_base = new FeatureBase(0); // FIXME unused variable ‘feature_base’
//
//
//    TimeStamp time_stamp;
//    time_stamp.setToNow();
//
//    std::vector<float> raw_data;
//    raw_data.push_back(42);
//    raw_data.push_back(43);
//    raw_data.push_back(44);
//
//
//
//    // Create sintetic gps capture
//    CaptureGPS* cpt_ptr_ = new CaptureGPS(time_stamp, gps_sensor_ptr_, raw_data);

//    // Add capture
//    wolf_manager_->addCapture(cpt_ptr_);
//
//    // update wolf tree
//    wolf_manager_->update();
//
//
//    wolf_manager_->getProblemPtr()->print();



    delete wolf_manager_; //not necessary to delete anything more, wolf will do it!

    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;

    //exit
    return 0;
}

