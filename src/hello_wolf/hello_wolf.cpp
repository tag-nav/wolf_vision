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
#include "ceres_wrapper/ceres_manager.h"
//#include "landmark_point_2D.h" // TODO this class does not exist yet

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
    params_odo->elapsed_time_th_  = 999;
    params_odo->dist_traveled_th_ = 0.95;
    params_odo->theta_traveled_th_= 999;
    params_odo->cov_det_th_       = 999;
    ProcessorBasePtr processor_odo  = problem->installProcessor("ODOM 2D", "processor odo", sensor_odo, params_odo);

    // sensor RB
    IntrinsicsRangeBearingPtr intrinsics_rb = std::make_shared<IntrinsicsRangeBearing>();
    intrinsics_rb->noise_bearing_degrees_std   = 1.0;
    intrinsics_rb->noise_range_metres_std      = 0.1;
    SensorBasePtr sensor_rb        = problem->installSensor("RANGE BEARING", "sensor RB", Vector3s(0,0,0), intrinsics_rb);

    // processor RB
    ProcessorParamsRangeBearingPtr params_rb = std::make_shared<ProcessorParamsRangeBearing>();
    params_rb->pose0 << 0,0,0;
    params_rb->delta << 1,0,0;
    ProcessorBasePtr processor_rb  = problem->installProcessor("RANGE BEARING", "processor RB", sensor_rb, params_rb);

//    problem->print();

    /* PROBLEM DEFINITION
     *
     * We consider 3 KFs and 3 lmks, observed as follows
     *
     *
     *      L1      L2      L3
     *      | \     | \     |
     *      |   \   |   \   |
     *      |     \ |     \ |
     *      F1->    F2->    F3->
     *
     * That is:
     *   - Lmks have ids '1', '2', '3'
     *   - All KFs look East, theta = 0
     *   - KFs  are at poses (0,0, 0), (1,0, 0), and (2,0, 0)
     *   - Lmks are at positions (0,1), (1,1), (2,1)
     *   - Observations have ranges 1 or sqrt(2)
     *   - Observations have bearings pi/2 or 3pi/4
     *
     * The sensor is considered at the origin (0,0, 0)
     */

    // Time
    TimeStamp t(0.0);

    // Motion
    Vector2s motion_data(1.0,0.0);
    Matrix2s motion_cov = 0.1 * Matrix2s::Identity();

    CaptureOdom2DPtr cap_motion = std::make_shared<CaptureOdom2D>(t, sensor_odo, motion_data, motion_cov);
    sensor_odo->process(cap_motion);


    return 0;
}
