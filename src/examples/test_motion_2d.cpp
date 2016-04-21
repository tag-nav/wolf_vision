/**
 * \file test_motion.cpp
 *
 *  Created on: Mar 15, 2016
 *      \author: jsola
 */

// Classes under test
#include "processor_odom_2D.h"

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
#include <iomanip>      // std::setprecision

int main()
{
    std::cout << std::setprecision(3);

    using namespace wolf;

    // time
    TimeStamp t0, t;
    t0.setToNow();
    t = t0;
    Scalar dt = .01;

    // Origin frame:
    Eigen::Vector2s p0 = Eigen::Vector2s::Zero();
    Eigen::Vector1s o0 = Eigen::Vector1s::Zero();
    Eigen::Vector3s x0;
    x0 << p0, o0;

//    // robot state blocks: origin
//    StateBlock robot_p_(Eigen::Vector2s::Zero());
//    StateBlock robot_o_(Eigen::Vector1s::Zero());

    // motion data
    Eigen::VectorXs data(2);
    data << 1, 0.1;  // advance 1m turn 0.1 rad (aprox 6 deg)
    Eigen::MatrixXs data_cov = Eigen::MatrixXs::Identity(2,2) * 0.01;

    // Create Wolf tree nodes
    Problem* problem_ptr = new Problem(FRM_PO_3D);
    SensorBase* sensor_ptr = new SensorBase(SEN_ODOM_2D, new StateBlock(Eigen::Vector2s::Zero()), new StateBlock(Eigen::Vector1s::Zero()), new StateBlock(Eigen::VectorXs::Zero(0)), 0);
    ProcessorOdom2d* odom2d_ptr = new ProcessorOdom2d();
    // Assemble Wolf tree by linking the nodes
    sensor_ptr->addProcessor(odom2d_ptr);
    problem_ptr->addSensor(sensor_ptr);

    // Initialize
    odom2d_ptr->setOrigin(x0, t0);

//    FrameBase* frm_ptr = new FrameBase(t, &robot_p_, &robot_o_);
//    std::cout << "frame created" << std::endl;
//    CaptureMotion2* cap_ptr = new CaptureMotion2(t, sensor_ptr, data, data_cov);
//    std::cout << "capture created" << std::endl;
//
//    frm_ptr->addCapture(cap_ptr);
//    std::cout << "capture added" << std::endl;

    std::cout << "Initial pose : " << problem_ptr->getLastFramePtr()->getState().transpose() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

    std::cout << "\nIntegrating states at synchronous time values..." << std::endl;

    std::cout << "State(" << (t-t0) << ") : " << odom2d_ptr->getState().transpose() << std::endl;
//    for (int i = 1; i <= 5; i++)
//    {
//        t += dt;
//        cap_ptr->setTimeStamp(t);
//        odom2d_ptr->process(cap_ptr);
//        std::cout << "State(" << (t-t0) << ") : " << odom2d_ptr->getState().transpose() << std::endl;
//    }
//
//    std::cout << "\nQuery states at asynchronous time values..." << std::endl;
//
//    t = t0;
//    Scalar dt_2 = dt/2;
//    dt = 0.0045; // new dt
//    for (int i = 1; i <= 20; i++)
//    {
//        std::cout << "State(" << (t-t0) << ") = " << odom2d_ptr->getState(t+dt_2).transpose() << std::endl;
//        t += dt;
//    }
//    std::cout << "       ^^^^^^^   After the last time-stamp the buffer keeps returning the last member." << std::endl;
//
//
//    return 0;
}
