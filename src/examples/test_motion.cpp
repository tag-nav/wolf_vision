/**
 * \file test_motion.cpp
 *
 *  Created on: Mar 15, 2016
 *      \author: jsola
 */


#include "processor_odom_3D.h"

#include "state_block.h"
#include "state_quaternion.h"
#include "capture_odom_3D.h"
#include "wolf.h"

#include <iostream>

int main()
{
    // time
    TimeStamp t0, t;
    t0.setToNow();
    t = t0;
    WolfScalar dt = .01;

    // robot state blocks: origin
    StateBlock sb_pos(Eigen::Vector3s::Zero());
    StateQuaternion sb_ori(Eigen::Quaternions::Identity().coeffs());

    // sensor state blocks: the same as robot
    StateBlock sb_intr(Eigen::VectorXs::Zero(0));

    // create sensor
    SensorBase* sensor_ptr = new SensorBase(ODOM_2D, &sb_pos, &sb_ori, &sb_intr, 0);

    // motion data
    Eigen::VectorXs data(6);
    data << 1, 0, 0,   // advance 1m
            0, 0, 0.1; // turn 0.1 rad (aprox 6 deg)


    std::cout << "Initial pose : " << sb_pos.getVector().transpose() << " " << sb_ori.getVector().transpose() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

    FrameBase* frm_ptr = new FrameBase(t, &sb_pos, &sb_ori);
    CaptureMotion2* cap_ptr = new CaptureOdom3D(t, sensor_ptr, data);
    frm_ptr->addCapture(cap_ptr);

    // Make a ProcessorOdom3d
    ProcessorOdom3d* odom3d_ptr = new ProcessorOdom3d(dt);
    sensor_ptr->addProcessor(odom3d_ptr);
    odom3d_ptr->init(cap_ptr);

    std::cout << "\nIntegrating states at synchronous time values..." << std::endl;

    for (int i = 1; i <= 5; i++)
    {
        cap_ptr->setTimeStamp(t);
        odom3d_ptr->process(cap_ptr);
        std::cout << "State(" << (t-t0) << ") : " << odom3d_ptr->state().transpose() << std::endl;
        t.set(t.get()+dt);
    }

    std::cout << "\nQuery states at asynchronous time values..." << std::endl;

    t = t0;
    dt = 0.0027;
    for (int i = 1; i <= 20; i++)
    {
        std::cout << "State(" << (t-t0) << ") = " << odom3d_ptr->state(t).transpose() << std::endl;
        t.set(t.get()+dt);
    }

    return 0;
}
