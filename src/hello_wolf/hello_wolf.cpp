/**
 * \file hello_wolf.cpp
 *
 *  Created on: Dec 1, 2017
 *      \author: jsola
 */


#include "wolf.h"

#include "sensor_odom_2D.h"
#include "processor_odom_2D.h"
#include "sensor_range_bearing.h"
#include "processor_range_bearing.h"
#include "capture_range_bearing.h"
#include "feature_range_bearing.h"
#include "constraint_range_bearing.h"
#include "landmark_point_2D.h"

#include "ceres_wrapper/ceres_manager.h"

int main()
{
    using namespace wolf;
    ProblemPtr problem                      = Problem::create("PO 2D");
    ceres::Solver::Options ceres_options;
    CeresManagerPtr ceres                   = std::make_shared<CeresManager>(problem, ceres_options);

    // sensor odom
    IntrinsicsOdom2DPtr intrinsics_odo = std::make_shared<IntrinsicsOdom2D>();
    intrinsics_odo->k_disp_to_disp  = 0.1;
    intrinsics_odo->k_rot_to_rot    = 0.1;
    SensorBasePtr sensor_odo        = problem->installSensor("ODOM 2D", "sensor odo", Vector3s(0,0,0), intrinsics_odo);

    // processor odom
    ProcessorParamsOdom2DPtr params_odo = std::make_shared<ProcessorParamsOdom2D>();
    params_odo->elapsed_time_th_    = 999;
    params_odo->dist_traveled_th_   = 0.95; // Will make KFs every 1m displacement
    params_odo->theta_traveled_th_  = 999;
    params_odo->cov_det_th_         = 999;
    ProcessorBasePtr processor      = problem->installProcessor("ODOM 2D", "processor odo", sensor_odo, params_odo);
    ProcessorOdom2DPtr processor_odo = std::static_pointer_cast<ProcessorOdom2D>(processor);

    // sensor RB
    IntrinsicsRangeBearingPtr intrinsics_rb     = std::make_shared<IntrinsicsRangeBearing>();
    intrinsics_rb->noise_bearing_degrees_std    = 1.0;
    intrinsics_rb->noise_range_metres_std       = 0.1;
    SensorBasePtr sensor_rb         = problem->installSensor("RANGE BEARING", "sensor RB", Vector3s(0,0,0), intrinsics_rb);

    // processor RB
    ProcessorParamsRangeBearingPtr params_rb = std::make_shared<ProcessorParamsRangeBearing>();
    ProcessorBasePtr processor_rb   = problem->installProcessor("RANGE BEARING", "processor RB", sensor_rb, params_rb);

    /* PROBLEM DEFINITION
     *
     * We consider 3 KFs and 3 lmks, observed as follows
     *
     *     (0,1)   (1,1)   (2,1)
     *      L1      L2      L3
     *      | \     | \     |
     *      |   \   |   \   |
     *      |     \ |     \ |
     *     KF1->   KF2->   KF3->
     *    (0,0,0) (1,0,0) (2,0,0)
     *
     * That is:
     *   - Lmks have ids '1', '2', '3'
     *   - All KFs look East, so all theta = 0
     *   - KFs  are at poses (0,0, 0), (1,0, 0), and (2,0, 0)
     *   - Lmks are at positions (0,1), (1,1), (2,1)
     *   - Observations have ranges 1 or sqrt(2)
     *   - Observations have bearings pi/2 or 3pi/4
     *
     * The sensor is considered at the robot's origin (0,0, 0)
     */

    // Origin
    TimeStamp   t(0.0);
    Vector3s    x(0,0,0);
    processor_odo->setOrigin(x, t);

    // Motion
    Vector2s motion_data(1.0,0.0);
    Matrix2s motion_cov = 0.1 * Matrix2s::Identity();

    CaptureOdom2DPtr cap_motion = std::make_shared<CaptureOdom2D>(t, sensor_odo, motion_data, motion_cov);

    // Set of events

    // observation
    VectorXi ids;
    VectorXs ranges, bearings;

    // STEP 1

    // lmks
    ids.resize(1); ranges.resize(1); bearings.resize(1);
    ids         << 1;
    ranges      << 1.0;
    bearings    << M_PI/2;
    CaptureRangeBearingPtr cap_rb = std::make_shared<CaptureRangeBearing>(t, sensor_rb, ids, ranges, bearings);
    sensor_rb   ->process(cap_rb);

    // motion
    t += 1.0;
    cap_motion  ->setTimeStamp(t);
    sensor_odo  ->process(cap_motion);

    // STEP 2

    // lmks
    ids.resize(2); ranges.resize(2); bearings.resize(2);
    ids         << 1, 2;
    ranges      << sqrt(2.0), 1.0;
    bearings    << 3*M_PI/4, M_PI/2;
    cap_rb      = std::make_shared<CaptureRangeBearing>(t, sensor_rb, ids, ranges, bearings);
    sensor_rb   ->process(cap_rb);

    // motion
    t += 1.0;
    cap_motion  ->setTimeStamp(t);
    sensor_odo  ->process(cap_motion);

    // STEP 3

    // lmks
    ids.resize(2); ranges.resize(2); bearings.resize(2);
    ids         << 2, 3;
    ranges      << sqrt(2.0), 1.0;
    bearings    << 3*M_PI/4, M_PI/2;
    cap_rb      = std::make_shared<CaptureRangeBearing>(t, sensor_rb, ids, ranges, bearings);
    sensor_rb   ->process(cap_rb);
    problem->print(4,1,1,1);

    // SOLVE with exact initial guess
    std::string report = ceres->solve(2);
    WOLF_TRACE(report);
    problem->print(4,1,1,1);

    // PERTURB initial guess and SOLVE again
    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
    {
        if (kf->id() == 1)
            kf->fix();
        else
            kf->setState(kf->getState() + 1 * Vector3s::Random());
    }
    for (auto lmk : problem->getMapPtr()->getLandmarkList())
        lmk->getPPtr()->setState(lmk->getPPtr()->getState() + 1 * Vector2s::Random());

    report = ceres->solve(2);
    WOLF_TRACE(report);
    problem->print(4,1,1,1);



    return 0;
}
