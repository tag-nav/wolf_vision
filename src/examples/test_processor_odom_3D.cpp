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

    //=====================================================
    // Environment variable for configuration files
    char const* tmp = std::getenv( "WOLF_ROOT" );
    if ( tmp == nullptr )
        throw std::runtime_error("WOLF_ROOT environment not loaded.");
    std::string wolf_path( tmp );
    std::cout << "Wolf path: " << wolf_path << std::endl;
    //=====================================================


    TimeStamp tf;
    if (argc == 1)
        tf = 1.0;
    else
    {

        tf.set(strtod(argv[1],nullptr));
    }
    cout << "Final timestamp tf = " << tf.get() << " s" << endl;

    ProblemPtr problem = Problem::create(FRM_PO_3D);
    CeresManager ceres_manager(problem);

    SensorBasePtr sen = problem->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_path + "/src/examples/odom_3D.yaml");
    problem->installProcessor("ODOM 3D", "odometry integrator", "odom", "");
    problem->getProcessorMotionPtr()->setOrigin((Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));

    Scalar dx = .1;
    Scalar dyaw = 2*M_PI/5;
    Vector6s data((Vector6s() << dx*cos(dyaw/2),dx*sin(dyaw/2),0,0,0,dyaw).finished()); // will integrate this data repeatedly

    Scalar dt = 0.1;

    CaptureMotion::Ptr cap_odo = std::make_shared<CaptureMotion>(TimeStamp(0), sen, data);

    cout << "t: " << std::setprecision(2) << 0 << "  \t x = ( " << problem->getCurrentState().transpose() << ")" << endl;

    for (TimeStamp t = dt; t < tf+dt/2; t += dt)
    {
        cap_odo->setTimeStamp(t);
        cap_odo->setData(data);
        cap_odo->process();

        cout << "t: " << std::setprecision(2) << t.get() << "  \t x = ( " << problem->getCurrentState().transpose() << ")" << endl;

        ceres::Solver::Summary summary = ceres_manager.solve();

//        ceres_manager.computeCovariances(ALL);

//        cout << summary.BriefReport() << endl;

    }

    problem->print();

    problem.reset();

    return 0;
}
