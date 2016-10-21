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
#include "map_base.h"
#include "landmark_base.h"
#include "ceres_wrapper/ceres_manager.h"

#include <cstdlib>

using namespace wolf;
using std::cout;
using std::endl;
using Eigen::Vector3s;
using Eigen::Vector6s;
using Eigen::Vector7s;
using Eigen::Quaternions;
using Eigen::VectorXs;



int main (int argc, char** argv)
{
    cout << "\n========= Test ProcessorOdom3D ===========" << endl;

    TimeStamp tf;
    if (argc == 1)
        tf = 0.3;
    else
    {

        tf.set(strtod(argv[1],nullptr));
    }
    cout << "Final timestamp tf = " << tf.get() << " s" << endl;

    ProblemPtr problem = Problem::create(FRM_PO_3D);
    CeresManager ceres_manager(problem);

    SensorBasePtr sen = problem->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(),"");
    problem->installProcessor("ODOM 3D", "odometry integrator", "odom", "");
    problem->getProcessorMotionPtr()->setOrigin((Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));

    Vector3s d_pos   ((Vector3s() << 0.1, 0, 0).finished());        // advance 0.1m
    Vector3s d_theta ((Vector3s() << 0, 0, M_PI/180).finished());   // turn 1 deg

    Vector6s data((Vector6s() << d_pos , d_theta).finished()); // will integrate this data repeatedly

    Scalar dt = 0.1;

    CaptureMotion::Ptr cap_odo = std::make_shared<CaptureMotion>(TimeStamp(0), sen, data);

    cout << "t: " << 0 << "  \t\t\t x = ( " << problem->getCurrentState().transpose() << ")" << endl;
    problem->print();
    cout << "--------------------------------------------------------------" << endl;

    for (TimeStamp t = dt; t < tf - Constants::EPS; t += dt)
    {
        cap_odo->setTimeStamp(t);
        cap_odo->setData(data);
        cap_odo->process();

        cout << "t: " << t.get() << "  \t\t x = ( " << problem->getCurrentState().transpose() << ")" << endl;
        problem->print();
        cout << "--------------------------------------------------------------" << endl;

        ceres::Solver::Summary summary = ceres_manager.solve();

//        ceres_manager.computeCovariances(ALL);

//        cout << summary.BriefReport() << endl;

    }

        //delete problem; // XXX Why is this throwing segfault?

    return 0;
}
