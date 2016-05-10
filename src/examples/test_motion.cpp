/**
 * \file test_motion.cpp
 *
 *  Created on: Mar 15, 2016
 *      \author: jsola
 */

// Classes under test
#include "processor_odom_3D.h"
#include "capture_motion2.h"

// Wolf includes
#include "state_block.h"
#include "state_quaternion.h"
#include "problem.h"
#include "wolf.h"

// STL includes
//#include <list>
//#include <algorithm>
//#include <iterator>

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
    Scalar dt = .01; // 100 Hz

    // Origin frame:
    Eigen::Vector3s pos(0.5, -0.5-sqrt(0.5), 0);
    Eigen::Quaternions quat(Eigen::AngleAxiss(M_PI_4, Eigen::Vector3s(0,0,1)));
    Eigen::Vector7s x0;
    x0 << pos, quat.coeffs();

    // motion data
    Eigen::VectorXs data(6);
    data << 1, 0, 0,   // advance 1m
            0, 0, M_PI_4; // 45 deg
    Eigen::MatrixXs data_cov = 0.1 * Eigen::MatrixXs::Identity(6,6);

    // Create Wolf tree nodes
    Problem* problem_ptr = new Problem(FRM_PO_3D);
    SensorBase* sensor_ptr = new SensorBase(SEN_ODOM_2D, new StateBlock(pos, true), new StateQuaternion(quat, true),
                                            new StateBlock(Eigen::VectorXs::Zero(0), true), 0);
    ProcessorOdom3D* odom3d_ptr = new ProcessorOdom3D();

    // Assemble Wolf tree by linking the nodes
    sensor_ptr->addProcessor(odom3d_ptr);
    problem_ptr->addSensor(sensor_ptr);

    // Initialize
    odom3d_ptr->setOrigin(x0, t0);

    std::cout << "Initial pose : " << pos.transpose() << " " << quat.coeffs().transpose() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

    std::cout << "\nIntegrating states at synchronous time values..." << std::endl;
    std::cout << "State(" << (t-t0) << ") : " << odom3d_ptr->getState().transpose() << std::endl;

    // Capture to use as container for all incoming data
    t += dt;
    CaptureMotion2* cap_ptr = new CaptureMotion2(t, sensor_ptr, data, data_cov);

    for (int i = 0; i <= 8; i++)
    {
        odom3d_ptr->process(cap_ptr);
        std::cout << "State(" << (t-t0) << ") : " << odom3d_ptr->getState().transpose() << std::endl;

        t += dt;
        cap_ptr->setTimeStamp(t);
    }

    std::cout << "\nQuery states at asynchronous time values..." << std::endl;

    t = t0;
    dt = 0.0045; // new dt
    for (int i = 1; i <= 25; i++)
    {
        std::cout << "State(" << (t-t0) << ") = " << odom3d_ptr->getState(t/*+dt/2*/).transpose() << std::endl;
        t += dt;
    }
    std::cout << "       ^^^^^^^   After the last time-stamp the buffer keeps returning the last member." << std::endl;



    // Split the buffer

    std::cout << "\nSplitting the buffer!\n---------------------" << std::endl;
    std::cout << "Original buffer:           < ";
    for (const auto &s : odom3d_ptr->getBufferPtr()->get() )
        std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;

    // first split at non-exact timestamp
    TimeStamp t_split = t0 + 0.033;
    std::cout << "Split time:                  " << t_split - t0 << std::endl;

    FrameBase* new_keyframe_ptr = problem_ptr->createFrame(KEY_FRAME,odom3d_ptr->getState(t_split), t_split);

    odom3d_ptr->keyFrameCallback(new_keyframe_ptr, 0);


    std::cout << "New buffer: oldest part:   < ";
    for (const auto &s : ((CaptureMotion2*)(new_keyframe_ptr->getCaptureListPtr()->front() ))->getBufferPtr()->get() )
        std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;

    std::cout << "Original keeps the newest: < ";
    for (const auto &s : odom3d_ptr->getBufferPtr()->get() )
        std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;

    std::cout << "All in one row:            < ";
    for (const auto &s : ((CaptureMotion2*)(new_keyframe_ptr->getCaptureListPtr()->front() ))->getBufferPtr()->get() )
        std::cout << s.ts_ - t0 << ' ';
    std::cout << "> " << t_split - t0 <<   " < ";
    for (const auto &s : odom3d_ptr->getBufferPtr()->get() )
        std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;

    // second split as exact timestamp

    t_split = t0 + 0.07;
    std::cout << "New split time:              " << t_split - t0 << std::endl;

    new_keyframe_ptr = problem_ptr->createFrame(KEY_FRAME,odom3d_ptr->getState(t_split), t_split);
    odom3d_ptr->keyFrameCallback(new_keyframe_ptr, 0);

    std::cout << "All in one row:            < ";
    for (const auto &s : ((CaptureMotion2*)(new_keyframe_ptr->getCaptureListPtr()->front() ))->getBufferPtr()->get() )
        std::cout << s.ts_ - t0 << ' ';
    std::cout << "> " << t_split - t0 <<   " < ";
    for (const auto &s : odom3d_ptr->getBufferPtr()->get() )
        std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;


    // Free allocated memory
    problem_ptr->destruct();

    return 0;
}
