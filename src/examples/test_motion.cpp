/**
 * \file test_motion.cpp
 *
 *  Created on: Mar 15, 2016
 *      \author: jsola
 */

// Classes under test
#include "processor_odom_3D.h"
#include "capture_odom_3D.h"
#include "problem.h"

// Wolf includes
#include "state_block.h"
#include "state_quaternion.h"
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
    Scalar dt = .01;

    // Origin frame:
    Eigen::Vector3s pos(2,2,0);
    Eigen::Quaternions quat(Eigen::AngleAxiss(3.1416/4, Eigen::Vector3s(0,0,1)));
    Eigen::Vector7s x0;
    x0 << pos, quat.coeffs();


    // robot state blocks: origin
    StateBlock sb_pos(pos);
    StateQuaternion sb_ori(quat);

    // sensor state blocks: the same as robot, plus intrinsic:
    StateBlock sb_intr(Eigen::VectorXs::Zero(0));

    // motion data
    Eigen::VectorXs data(6);
    data << 1, 0, 0,   // advance 1m
            0, 0, 3.1416/4; // 45 deg


    // Create Wolf tree nodes
    Problem* problem_ = new Problem(FRM_PO_3D);

    SensorBase* sensor_ptr = new SensorBase(SEN_ODOM_2D, &sb_pos, &sb_ori, &sb_intr, 0);

    ProcessorOdom3d* odom3d_ptr = new ProcessorOdom3d(dt);

    // Assemble Wolf tree by linking the nodes
    sensor_ptr->addProcessor(odom3d_ptr);

    problem_->addSensor(sensor_ptr);

    // Initialize
    odom3d_ptr->setOrigin(x0, new CaptureOdom3D(dt, sensor_ptr));

    std::cout << "Initial pose : " << sb_pos.getVector().transpose() << " " << sb_ori.getVector().transpose() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

    // New Capture
    CaptureMotion2* cap_ptr = new CaptureOdom3D(t, sensor_ptr, data);

    std::cout << "\nIntegrating states at synchronous time values..." << std::endl;

    for (int i = 0; i <= 8; i++)
    {
        odom3d_ptr->process(cap_ptr);
        std::cout << "State(" << (t-t0) << ") : " << odom3d_ptr->getState().transpose() << std::endl;

        t += dt;
        cap_ptr->setTimeStamp(t);
    }

    std::cout << "\nQuery states at asynchronous time values..." << std::endl;

    t = t0;
    Scalar dt_2 = dt/2;
    dt = 0.0045; // new dt
    for (int i = 1; i <= 25; i++)
    {
        std::cout << "State(" << (t-t0) << ") = " << odom3d_ptr->getState(t+dt_2).transpose() << std::endl;
        t += dt;
    }
    std::cout << "       ^^^^^^^   After the last time-stamp the buffer keeps returning the last member." << std::endl;



    // Split the buffer
    MotionBuffer oldest_buffer;

    std::cout << "\nSplitting the buffer!\n---------------------" << std::endl;
    std::cout << "Original buffer:           < ";
    for (const auto &s : cap_ptr->getBufferPtr()->getContainer() ) std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;

    TimeStamp t_split = t0 + 0.033;
    std::cout << "Split time:                  " << t_split - t0 << std::endl;

    odom3d_ptr->splitBuffer(t_split, oldest_buffer);


    std::cout << "New buffer: oldest part:   < ";
    for (const auto &s : oldest_buffer.getContainer() ) std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;

    std::cout << "Original keeps the newest: < ";
    for (const auto &s : odom3d_ptr->getBufferPtr()->getContainer() ) std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;

    std::cout << "All in one row:            < ";
    for (const auto &s : oldest_buffer.getContainer() ) std::cout << s.ts_ - t0 << ' ';
    std::cout << "> " << t_split - t0 <<   " < ";
    for (const auto &s : odom3d_ptr->getBufferPtr()->getContainer() ) std::cout << s.ts_ - t0 << ' ';
    std::cout << ">" << std::endl;







#if 0  // Skip this part, it's only preliminary tests


    std::cout << "\n\nTrying a std::map as the buffer container <-- NOT WORKING: need exact key" << std::endl;

    Scalar x;
    std::map<TimeStamp, Scalar> buffer_map;
    t.set(0);
    x = 0;
    for (double i = 1; i<=10; i++)
    {
        t.set(i/5);
        x++;
        buffer_map.insert(std::pair<TimeStamp,Scalar>(t,x));
        std::cout << "insert (ts,x) = (" << t.get() << "," << x << ")" << std::endl;
    }
    for (double i = 1; i<=8; i++)
    {
        t.set(i/4);
        std::cout << "query (" << t.get() << "," << buffer_map[t] << ")" << std::endl;
    }





    std::cout << "\n\nTrying a std::list and std::find_if as the buffer container <-- WORKING: can use comparator '<' for evaluating key" << std::endl;

    typedef std::pair<TimeStamp, Scalar> Pair;
    typedef std::list<Pair> PairsList;

    PairsList buffer_list;
    t.set(0);
    x = 0;
    for (double i = 0; i<=8; i++)
    {
        t.set(i/4);
        x++;
        buffer_list.push_back(Pair(t,x));
        std::cout << "insert (ts,x) = (" << t.get() << "," << x << ")" << std::endl;
    }

    std::cout << "\nFinding in direct order..." << std::endl;

    PairsList::iterator it_next;
    for (double i = 1; i<=10; i++)
    {
        t.set(i/5);
        int n = 0;
        it_next = std::find_if (buffer_list.begin(), buffer_list.end(), [&](const Pair& p){n++;return t <= p.first;});

        assert(it_next != buffer_list.end() && "Buffer data not found for the provided time stamp.");

        std::cout << n << " query " << t.get() << "-> previous: (" << std::prev(it_next)->first.get() << "," << std::prev(it_next)->second << "); NEXT: (" << it_next->first.get() << "," << it_next->second << ")" << std::endl;
    }

    std::cout << "\nFinding in reverse order..." << std::endl;

    PairsList::reverse_iterator it_previous;
    for (double i = 1; i<=10; i++)
    {
        t.set(i/5);
        int n = 0;
        it_previous = std::find_if (buffer_list.rbegin(), buffer_list.rend(), [&](const Pair& p){
            n++;
            return p.first <= t;});

        assert(it_previous != buffer_list.rend() && "Buffer data not found for the provided time stamp.");

        std::cout << n << " query " << t.get() << "-> PREVIOUS: (" << it_previous->first.get() << "," << it_previous->second << "); next: (" << std::prev(it_previous)->first.get() << "," << std::prev(it_previous)->second << ")" << std::endl;
    }

#endif


    return 0;
}
