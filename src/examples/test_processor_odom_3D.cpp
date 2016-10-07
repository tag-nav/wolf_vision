/**
 * \file test_processor_odom_3D.cpp
 *
 *  Created on: Oct 7, 2016
 *      \author: jsola
 */


#include "problem.h"
#include "sensor_odom_2D.h"
#include "processor_odom_3D.h"

using namespace wolf;
using std::cout;
using std::endl;
using Eigen::Vector3s;
using Eigen::Vector7s;
using Eigen::Quaternions;
using Eigen::VectorXs;

int main ()
{

    Problem* problem = new Problem(FRM_PO_3D);

    SensorBase* sen = problem->installSensor("ODOM 2D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(),"");
    ProcessorBase* prc = problem->installProcessor("ODOM 3D", "odometry integrator", "odom", "");

    Vector3s d_pos   ((Vector3s() << 0,0,0).finished());
    Vector3s d_theta ((Vector3s() << 0,0,0).finished());

    Scalar dt = 0.001;

    for (TimeStamp t = 0; t < 5; t += dt)
    {
        cout << t.get() << endl;

    }

    delete problem;
    return 0;
}
