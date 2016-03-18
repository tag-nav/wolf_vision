/**
 * \file test_motion.cpp
 *
 *  Created on: Mar 15, 2016
 *      \author: jsola
 */


#include "processor_odom_3D.h"

#include "state_block.h"
#include "state_quaternion.h"
#include "capture_motion2.h"
#include "wolf.h"

class CaptureOdom3D : public CaptureMotion2
{
    public:
        CaptureOdom3D(const TimeStamp& _ts, SensorBase* _sensor_ptr,
                      const Eigen::Vector6s& _data) :
                CaptureMotion2(_ts, _sensor_ptr, _data)
        {
            //
        }

        // TODO This needs to go out!
    public:
        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now) const {return Eigen::VectorXs::Zero(7);}


};





#include <iostream>

int main()
{
    // time
    TimeStamp t;
    t.setToNow();

    // robot state blocks: origin
    StateBlock sb_pos(Eigen::Vector3s::Zero());
    StateQuaternion sb_ori(Eigen::Quaternions::Identity().coeffs());

    // sensor state blocks: the same as robot
    StateBlock sb_intr(Eigen::VectorXs::Zero(0));

    // create sensor
    SensorBase sensor(ODOM_2D, &sb_pos, &sb_ori, &sb_intr, 0);

    // motion data
    Eigen::VectorXs data(6);
    data << 1, 2, 3, 4, 5, 6;


    std::cout << "Initial pose : " << sb_pos.getVector().transpose() << " " << sb_ori.getVector().transpose() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

    FrameBase* frm_ptr = new FrameBase(t, &sb_pos, &sb_ori);
    CaptureMotion2* cap_ptr = new CaptureOdom3D(t, &sensor, data);
    frm_ptr->addCapture(cap_ptr);

    // Make a ProcessorOdom3d
    ProcessorOdom3d odom3d(0.1);
    odom3d.init(cap_ptr);
//    std::cout << "Initial pose : " << odom3d.x_origin_.transpose() << " " << sb_ori.getVector().transpose() << std::endl;
//    std::cout << "Motion data  : " << odom3d.data_.transpose() << std::endl;
//    std::cout << "Motion delta : " << odom3d.dx_.transpose() << std::endl;
    std::cout << "State : " << odom3d.state(t).transpose() << std::endl;

    // Add a capture to process
    odom3d.process(cap_ptr);
    std::cout << "State : " << odom3d.state().transpose() << std::endl;

//    std::cout << "Initial pose : " << odom3d.x_origin_.transpose() << " " << sb_ori.getVector().transpose() << std::endl;
//    std::cout << "Motion data  : " << odom3d.data_.transpose() << std::endl;
//    std::cout << "Motion delta : " << odom3d.dx_.transpose() << std::endl;
//    std::cout << "Delta integr : " << odom3d.Dx_integral_.transpose() << std::endl;


    return 0;
}
