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

    std::string wolf_root = _WOLF_ROOT_DIR;

    TimeStamp tf;
    if (argc == 1)
        tf = 1.0;
    else
    {

        tf.set(strtod(argv[1],nullptr));
    }
    cout << "Final timestamp tf = " << tf.get() << " s" << endl;

    ProblemPtr problem = Problem::create("PO 3D");
    ceres::Solver::Options ceres_options;
//    ceres_options.max_num_iterations = 1000;
//    ceres_options.function_tolerance = 1e-10;
//    ceres_options.gradient_check_relative_precision = 1e-10;
//    ceres_options.gradient_tolerance = 1e-10;
    ceres_options.minimizer_progress_to_stdout = true;
    CeresManager ceres_manager(problem, ceres_options);

    SensorBasePtr sen = problem->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    problem->installProcessor("ODOM 3D", "odometry integrator", "odom");
    problem->getProcessorMotionPtr()->setOrigin((Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));

    Scalar dx = .1;
    Scalar dyaw = 2*M_PI/5;
    Vector6s data((Vector6s() << dx*cos(dyaw/2),dx*sin(dyaw/2),0,0,0,dyaw).finished()); // will integrate this data repeatedly

    Scalar dt = 0.1;

    CaptureMotionPtr cap_odo = std::make_shared<CaptureMotion>(TimeStamp(0), sen, data, 7, 7, 6, 0 );

    cout << "t: " << std::setprecision(2) << 0 << "  \t x = ( " << problem->getCurrentState().transpose() << ")" << endl;

    for (TimeStamp t = dt; t < tf+dt/2; t += dt)
    {
        cap_odo->setTimeStamp(t);
        cap_odo->setData(data);

        sen->process(cap_odo);

        cout << "t: " << std::setprecision(2) << t.get() << "  \t x = ( " << problem->getCurrentState().transpose() << ")" << endl;

//        ceres::Solver::Summary summary = ceres_manager.solve();

//        ceres_manager.computeCovariances(ALL);

//        cout << summary.BriefReport() << endl;

    }

    problem->print(1,0,1,0);
//    for (auto frm : problem->getTrajectoryPtr()->getFrameList())
//    {
//        frm->setState(problem->zeroState());
//    }
//    problem->print(1,0,1,0);
    std::string brief_report = ceres_manager.solve(1);// 0, 1 or 2
    std::cout << brief_report << std::endl;
    problem->print(1,0,1,0);

    problem.reset();

    return 0;
}
