/**
 * \file test_motion.cpp
 *
 *  Created on: Mar 15, 2016
 *      \author: jsola
 */

// Classes under test
#include "processor_odom_2D.h"
#include "capture_odom_2D2.h"

// Wolf includes
#include "state_block.h"
#include "wolf.h"

// STL includes
#include <map>
#include <list>
#include <algorithm>
#include <iterator>

// General includes
#include <iostream>

int main()
{
    using namespace wolf;

    // time
    TimeStamp t0, t;
    t0.setToNow();
    t = t0;
    WolfScalar dt = .01;

    // robot state blocks: origin
    StateBlock robot_p_(Eigen::Vector2s::Zero());
    StateBlock robot_o_(Eigen::Vector1s::Zero());

    // create sensor
    SensorBase* sensor_ptr = new SensorBase(SEN_ODOM_2D, new StateBlock(Eigen::Vector2s::Zero()), new StateBlock(Eigen::Vector1s::Zero()), new StateBlock(Eigen::VectorXs::Zero(0)), 0);

    // motion data
    Eigen::VectorXs data(2);
    data << 1, 0.1;  // advance 1m turn 0.1 rad (aprox 6 deg)

    std::cout << "Initial pose : " << robot_p_.getVector().transpose() << " " << robot_o_.getVector().transpose() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

    FrameBase* frm_ptr = new FrameBase(t, &robot_p_, &robot_o_);
    std::cout << "frame created" << std::endl;
    CaptureMotion2* cap_ptr = new CaptureOdom2D2(t, sensor_ptr, data);
    std::cout << "capture created" << std::endl;

    frm_ptr->addCapture(cap_ptr);
    std::cout << "capture added" << std::endl;

    // Make a ProcessorOdom3d
    ProcessorOdom2d* odom2d_ptr = new ProcessorOdom2d(dt);
    sensor_ptr->addProcessor(odom2d_ptr);
    odom2d_ptr->init(cap_ptr);

    std::cout << "\nIntegrating states at synchronous time values..." << std::endl;

    std::cout << "State(" << (t-t0) << ") : " << odom2d_ptr->state().transpose() << std::endl;
    for (int i = 1; i <= 5; i++)
    {
        t += dt;
        cap_ptr->setTimeStamp(t);
        odom2d_ptr->process(cap_ptr);
        std::cout << "State(" << (t-t0) << ") : " << odom2d_ptr->state().transpose() << std::endl;
    }

    std::cout << "\nQuery states at asynchronous time values..." << std::endl;

    t = t0;
    WolfScalar dt_2 = dt/2;
    dt = 0.0045; // new dt
    for (int i = 1; i <= 20; i++)
    {
        std::cout << "State(" << (t-t0) << ") = " << odom2d_ptr->state(t+dt_2).transpose() << std::endl;
        t += dt;
    }
    std::cout << "       ^^^^^^^   After the last time-stamp the buffer keeps returning the last member." << std::endl;


    return 0;
}
