/*
 * gtest_capture_base.cpp
 *
 *  Created on: Feb 15, 2018
 *      Author: asantamaria
 */

//Wolf
#include "utils_gtest.h"

#include "processor_odom_2D.h"
#include "sensor_odom_2D.h"

#include "processor_tracker_feature_dummy.h"
#include "capture_void.h"

#include "problem.h"

// STL
#include <iterator>
#include <iostream>

using namespace wolf;
using namespace Eigen;


TEST(ProcessorBase, KeyFrameCallback)
{

    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
    using Eigen::Vector2s;

    Scalar dt = 0.01;

    // Wolf problem
    ProblemPtr problem = Problem::create("PO 2D");

    // Install tracker (sensor and processor)
    SensorBasePtr sens_trk = make_shared<SensorBase>("FEATURE", std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)),
                                                     std::make_shared<StateBlock>(Eigen::VectorXs::Zero(1)),
                                                     std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), 2);
    shared_ptr<ProcessorTrackerFeatureDummy> proc_trk = make_shared<ProcessorTrackerFeatureDummy>(5, 5);
    proc_trk->setTimeTolerance(dt/2);

    problem->addSensor(sens_trk);
    sens_trk->addProcessor(proc_trk);

    // Install odometer (sensor and processor)
    SensorBasePtr sens_odo = problem->installSensor("ODOM 2D", "odometer", Vector3s(0,0,0), "");
    ProcessorParamsOdom2DPtr proc_odo_params = make_shared<ProcessorParamsOdom2D>();
    ProcessorBasePtr proc_odo = problem->installProcessor("ODOM 2D", "odometer", sens_odo, proc_odo_params);
    proc_odo->setTimeTolerance(dt/2);

    std::cout << "sensor & processor created and added to wolf problem" << std::endl;

    // Sequence to test KeyFrame creations (callback calls)

    // initialize
    TimeStamp   t(0.0);
    Vector3s    x(0,0,0);
    Matrix3s    P = Matrix3s::Identity() * 0.1;
    problem->setPrior(x, P, t, dt/2);             // KF1

    CaptureOdom2DPtr capt_odo = make_shared<CaptureOdom2D>(t, sens_odo, Vector2s(0.5,0));

    // Track
    CaptureVoidPtr capt_trk(make_shared<CaptureVoid>(t, sens_trk));
    proc_trk->process(capt_trk);

    for (size_t ii=0; ii<10; ii++ )
    {
        // Move
        t = t+dt;
        WOLF_INFO("----------------------- ts: ", t , " --------------------------");

        capt_odo->setTimeStamp(t);
        proc_odo->process(capt_odo);

        // Track
        capt_trk = make_shared<CaptureVoid>(t, sens_trk);
        proc_trk->process(capt_trk);

//        problem->print(4,1,1,0);

        // Only odom creating KFs
        ASSERT_TRUE( problem->getLastKeyFramePtr()->getType().compare("PO 2D")==0 );
    }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

