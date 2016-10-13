/**
 * \file test_processor_odom_3D.cpp
 *
 *  Created on: Oct 7, 2016
 *      \author: jsola
 */


#include "problem.h"
#include "sensor_odom_2D.h"
#include "processor_odom_3D.h"
#include "capture_imu.h"

using namespace wolf;
using std::cout;
using std::endl;
using Eigen::Vector3s;
using Eigen::Vector6s;
using Eigen::Vector7s;
using Eigen::Quaternions;
using Eigen::VectorXs;

int main ()
{

    Problem* problem = new Problem(FRM_PO_3D);

    SensorBase* sen = problem->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(),"");
    problem->installProcessor("ODOM 3D", "odometry integrator", "odom", "");
    problem->getProcessorMotionPtr()->setOrigin((Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));

    Vector3s d_pos   ((Vector3s() << 0.1, 0.2, 0.3).finished());
    Vector3s d_theta ((Vector3s() << 0.01,0.02,0.03).finished());

    Vector6s data((Vector6s() << d_pos , d_theta).finished());

    Scalar dt = 0.01;

    for (TimeStamp t = 0; t < 5 - Constants::EPS; t += dt)
    {

        CaptureMotion* cap_odo = new CaptureMotion(t, sen, data);

        cap_odo->process();

        cout << "t: " << t.get() << "   x: " << problem->getCurrentState().transpose() << endl;

    }

    delete problem;
    return 0;
}
