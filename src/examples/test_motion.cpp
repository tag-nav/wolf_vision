/**
 * \file test_motion.cpp
 *
 *  Created on: Mar 15, 2016
 *      \author: jsola
 */

// Classes under test
#include "processor_odom_3D.h"
#include "capture_odom_3D.h"

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
    SensorBase* sensor_ptr = new SensorBase(SEN_ODOM_2D, &sb_pos, &sb_ori, &sb_intr, 0);

    // motion data
    Eigen::VectorXs data(6);
    data << 1, 0, 0,   // advance 1m
            0, 0, 0.1; // turn 0.1 rad (aprox 6 deg)


    std::cout << "Initial pose : " << sb_pos.getVector().transpose() << " " << sb_ori.getVector().transpose() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

    FrameBase* frm_ptr = new FrameBase(t, &sb_pos, &sb_ori);
    CaptureMotion2<Odo3dDeltaType>* cap_ptr = new CaptureOdom3D(t, sensor_ptr, data);
    frm_ptr->addCapture(cap_ptr);

    // Make a ProcessorOdom3d
    ProcessorOdom3d* odom3d_ptr = new ProcessorOdom3d(dt);
    sensor_ptr->addProcessor(odom3d_ptr);
    odom3d_ptr->init(cap_ptr);

    std::cout << "\nIntegrating states at synchronous time values..." << std::endl;

    std::cout << "State(" << (t-t0) << ") : " << odom3d_ptr->state().transpose() << std::endl;
    for (int i = 1; i <= 5; i++)
    {
        t += dt;
        cap_ptr->setTimeStamp(t);
        odom3d_ptr->process(cap_ptr);
        std::cout << "State(" << (t-t0) << ") : " << odom3d_ptr->state().transpose() << std::endl;
    }

    std::cout << "\nQuery states at asynchronous time values..." << std::endl;

    t = t0;
    WolfScalar dt_2 = dt/2;
    dt = 0.0045; // new dt
    for (int i = 1; i <= 20; i++)
    {
        std::cout << "State(" << (t-t0) << ") = " << odom3d_ptr->state(t+dt_2).transpose() << std::endl;
        t += dt;
    }
    std::cout << "       ^^^^^^^   After the last time-stamp the buffer keeps returning the last member." << std::endl;






    std::cout << "\n\nTrying a std::map as the buffer container <-- NOT WORKING: need exact key" << std::endl;

    WolfScalar x;
    std::map<TimeStamp, WolfScalar> buffer_map;
    t.set(0);
    x = 0;
    for (double i = 1; i<=10; i++)
    {
        t.set(i/5);
        x++;
        buffer_map.insert(std::pair<TimeStamp,WolfScalar>(t,x));
        std::cout << "insert (ts,x) = (" << t.get() << "," << x << ")" << std::endl;
    }
    for (double i = 1; i<=8; i++)
    {
        t.set(i/4);
        std::cout << "query (" << t.get() << "," << buffer_map[t] << ")" << std::endl;
    }





    std::cout << "\n\nTrying a std::list and std::find_if as the buffer container <-- WORKING: can use comparator '<' for evaluating key" << std::endl;

    typedef std::pair<TimeStamp, WolfScalar> Pair;
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


    return 0;
}
