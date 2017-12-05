/**
 * \file hello_wolf.cpp
 *
 *  Created on: Dec 1, 2017 -- two months exactly after Oct-1st, we still have 4 political prisoners.
 *
 *     ###
 *    ## ##
 *     ###
 *    ## ##
 *   ##   ##
 *
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
    /* PROBLEM DEFINITION
     *
     * We consider 3 keyframes 'KF' and 3 landmarks 'L', observed as follows
     *
     *     (0,1)   (1,1)   (2,1)
     *      L1      L2      L3
     *      | \     | \     |
     *      |   \   |   \   |
     *      |     \ |     \ |
     *     KF1->---KF2->---KF3->
     *    (0,0,0) (1,0,0) (2,0,0)
     *      |
     *      |
     *      * prior
     *    (0,0,0)
     *
     * where the links '--', '\' and '|' are the measurement factors.
     *
     * That is:
     *   - Lmks have ids '1', '2', '3'
     *   - All KFs look East, so all theta = 0
     *   - KFs  are at poses (0,0, 0), (1,0, 0), and (2,0, 0)
     *   - Lmks are at positions (0,1), (1,1), (2,1)
     *   - The sensor is considered at the robot's origin (0,0, 0)
     *   - Observations have ranges 1 or sqrt(2)
     *   - Observations have bearings pi/2 or 3pi/4
     *   - We set a prior at (0,0,0) on KF1 to render the system observable
     */

    // SET PROBLEM =======================================================

    using namespace wolf;

    // Wolf problem and solver
    ProblemPtr problem                      = Problem::create("PO 2D");
    ceres::Solver::Options ceres_options;
    ceres_options.max_num_iterations        = 1000; // We depart far from solution, need a lot of iterations
    CeresManagerPtr ceres                   = std::make_shared<CeresManager>(problem, ceres_options);

    // sensor odometer 2D
    IntrinsicsOdom2DPtr intrinsics_odo  = std::make_shared<IntrinsicsOdom2D>();
    intrinsics_odo->k_disp_to_disp      = 0.1;
    intrinsics_odo->k_rot_to_rot        = 0.1;
    SensorBasePtr sensor_odo            = problem->installSensor("ODOM 2D", "sensor odo", Vector3s(0,0,0), intrinsics_odo);

    // processor odometer 2D
    ProcessorParamsOdom2DPtr params_odo = std::make_shared<ProcessorParamsOdom2D>();
    params_odo->elapsed_time_th_        = 999;
    params_odo->dist_traveled_th_       = 0.95; // Will make KFs automatically every 1m displacement
    params_odo->theta_traveled_th_      = 999;
    params_odo->cov_det_th_             = 999;
    params_odo->unmeasured_perturbation_std_ = 0.001;
    ProcessorBasePtr processor          = problem->installProcessor("ODOM 2D", "processor odo", sensor_odo, params_odo);
    ProcessorOdom2DPtr processor_odo    = std::static_pointer_cast<ProcessorOdom2D>(processor);

    // sensor Range and Bearing
    IntrinsicsRangeBearingPtr intrinsics_rb     = std::make_shared<IntrinsicsRangeBearing>();
    intrinsics_rb->noise_bearing_degrees_std    = 1.0;
    intrinsics_rb->noise_range_metres_std       = 0.1;
    SensorBasePtr sensor_rb             = problem->installSensor("RANGE BEARING", "sensor RB", Vector3s(0,0,0), intrinsics_rb);

    // processor Range and Bearing
    ProcessorParamsRangeBearingPtr params_rb = std::make_shared<ProcessorParamsRangeBearing>();
    params_rb->time_tolerance           = 0.01;
    ProcessorBasePtr processor_rb       = problem->installProcessor("RANGE BEARING", "processor RB", sensor_rb, params_rb);


    // CONFIGURE ==========================================================

    // Motion data
    Vector2s motion_data(1.0, 0.0);
    Matrix2s motion_cov = 0.1 * Matrix2s::Identity();

    // landmark observations data
    VectorXi ids;
    VectorXs ranges, bearings;


    // SET OF EVENTS =======================================================
    // We'll do 3 steps of motion and landmark observations.

    // STEP 1 --------------------------------------------------------------

    // initialize
    TimeStamp   t(0.0);
    Vector3s    x(0,0,0);
    Matrix3s    P = Matrix3s::Identity() * 0.1;
    problem->setPrior(x, P, t);             // KF1

    // observe lmks
    ids.resize(1); ranges.resize(1); bearings.resize(1);
    ids         << 1;                       // will observe Lmk 1
    ranges      << 1.0;                     // see drawing
    bearings    << M_PI/2;
    CaptureRangeBearingPtr cap_rb = std::make_shared<CaptureRangeBearing>(t, sensor_rb, ids, ranges, bearings);
    sensor_rb   ->process(cap_rb);

    // STEP 2 --------------------------------------------------------------
    t += 1.0;

    // motion
    CaptureOdom2DPtr cap_motion = std::make_shared<CaptureOdom2D>(t, sensor_odo, motion_data, motion_cov);
    sensor_odo  ->process(cap_motion);      // KF2

    // observe lmks
    ids.resize(2); ranges.resize(2); bearings.resize(2);
    ids         << 1, 2;                    // will observe Lmks 1 and 2
    ranges      << sqrt(2.0), 1.0;          // see drawing
    bearings    << 3*M_PI/4, M_PI/2;
    cap_rb      = std::make_shared<CaptureRangeBearing>(t, sensor_rb, ids, ranges, bearings);
    sensor_rb   ->process(cap_rb);

    // STEP 3 --------------------------------------------------------------
    t += 1.0;

    // motion
    cap_motion  ->setTimeStamp(t);
    sensor_odo  ->process(cap_motion);      // KF3

    // observe lmks
    ids.resize(2); ranges.resize(2); bearings.resize(2);
    ids         << 2, 3;                    // will observe Lmks 2 and 3
    ranges      << sqrt(2.0), 1.0;          // see drawing
    bearings    << 3*M_PI/4, M_PI/2;
    cap_rb      = std::make_shared<CaptureRangeBearing>(t, sensor_rb, ids, ranges, bearings);
    sensor_rb   ->process(cap_rb);
    problem->print(4,1,1,1);


    // SOLVE ================================================================

    // SOLVE with exact initial guess
    std::string report = ceres->solve(2);
    WOLF_TRACE(report);                     // should show a very low iteration number (possibly 1)
    problem->print(4,1,1,1);

    // PERTURB initial guess
    for (auto kf : problem->getTrajectoryPtr()->getFrameList())
        kf->setState(Vector3s::Random());                       // We perturb A LOT !
    for (auto lmk : problem->getMapPtr()->getLandmarkList())
        lmk->getPPtr()->setState(Vector2s::Random());           // We perturb A LOT !

    // SOLVE again
    report = ceres->solve(2);
    WOLF_TRACE(report);                     // should show a very high iteration number (more than 10, or than 100!)
    problem->print(4,1,1,1);
    /*
     * Note:
     *
     * IF YOU SEE at the end of the printed problem the estimate for Lmk 3 as:
     *
     * L3 POINT 2D   <-- c8
     *   Est,     x = ( 2 1)
     *   sb: Est
     *
     * it means WOLF SOLVED SUCCESSFULLY (L3 is effectively at location (2,1) ) !
     *
     * Side notes:
     *
     *  - Observe that all other KFs and Lmks are correct.
     *
     *  - Observe that F4 is not correct. Since it is not a KF, is has not been estimated.
     *    But this is a no-issue because F4 is just an inner frame used by the odometer processor,
     *    with no role in the problem itself, nor in the optimization process.
     *
     */


    /*
     * Note: description of the printed result.
     *
     * The line problem->print(4,1,1,1) produce a printout of the status of the WOLF problem.
     * The full message is explained below.
     *
     * P: wolf tree status ---------------------
        Hardware
          S1 ODOM 2D [Sta,Sta]                          // Sensor 1, type ODOMETRY 2D, static extrinsics and intrinsics
            sb: Fix Fix                                 // Extrinsics position and orientation are fixed. No intrinsics.
            pm1 ODOM 2D                                 // Processor 1, type ODOMETRY 2D
              o: C7 - F3                                // origin at Capture 7, Frame 3
              l: C10 - F4                               // last at Capture 10, frame 4
          S2 RANGE BEARING [Sta,Sta]                    // Sensor 2, type RANGE and BEARING, static extrinsics and intrinsics
            sb: Fix Fix                                 // Fixed position and orientation. No intrinsics.
            pt2 RANGE BEARING                           // Processor 2: type Range and Bearing
        Trajectory
          KF1  <-- c3                                   // KeyFrame 1, constrained by Constraint 3
            Est, ts=0,   x = ( -1.6e-13 9.4e-11  1.4e-10 ) // State is estimated; time stamp and state vector
            sb: Est Est                                 // State's pos and orient are estimated
            C1 FIX -> S- [  <--                         // Capture 1, type FIX or Absolute
              f1 FIX  <--                               // Feature 1, type Fix
                m = ( 0 0 0)                            // The absolute measurement for this frame is (0,0,0) --> origin
                c1 FIX --> A                            // Constraint 1, type FIX, it is Absolute
            CM2 ODOM 2D -> S1 [Sta, Sta]  <--           // Capture 2, type ODOM, from Sensor 1 (static extr and intr)
            C5 RANGE BEARING -> S2 [Sta, Sta]  <--      // Capture 5, type RB, from Sensor 2 (static extr and intr)
              f2 RANGE BEARING  <--                     // Feature 2, type RB
                m = ( 1    1.57)                        // The feature's measurement is 1m, 1.57rad
                c2 RANGE BEARING --> L1                 // Constraint 2 against Landmark 1
          KF2  <-- c6
            Est, ts=1,   x = ( 1       2.5e-10 1.6e-10)
            sb: Est Est
            CM3 ODOM 2D -> S1 [Sta, Sta]  <--
              f3 ODOM 2D  <--
                m = ( 1 0 0)
                c3 ODOM 2D --> F1                       // Constraint 3, type ODOM, against Frame 1
            C9 RANGE BEARING -> S2 [Sta, Sta]  <--
              f4 RANGE BEARING  <--
                m = ( 1.41 2.36)
                c4 RANGE BEARING --> L1
              f5 RANGE BEARING  <--
                m = ( 1    1.57)
                c5 RANGE BEARING --> L2
          KF3  <--
            Est, ts=2,   x = ( 2       4.1e-10 1.7e-10)
            sb: Est Est
            CM7 ODOM 2D -> S1 [Sta, Sta]  <--
              f6 ODOM 2D  <--
                m = ( 1 0 0)
                c6 ODOM 2D --> F2
            C12 RANGE BEARING -> S2 [Sta, Sta]  <--
              f7 RANGE BEARING  <--
                m = ( 1.41 2.36)
                c7 RANGE BEARING --> L2
              f8 RANGE BEARING  <--
                m = ( 1    1.57)
                c8 RANGE BEARING --> L3
          F4  <--
            Est, ts=2,   x = ( 0.11   -0.045 0.26  )
            sb: Est Est
            CM10 ODOM 2D -> S1 [Sta, Sta]  <--
        Map
          L1 POINT 2D   <-- c2  c4                      // Landmark 1, constrained by Constraints 2 and 4
            Est,     x = ( -1.4e-10 1       )           // L4 state is estimated, state vector
            sb: Est                                     // L4 has 1 state block estimated
          L2 POINT 2D   <-- c5  c7
            Est,     x = ( 1 1)
            sb: Est
          L3 POINT 2D   <-- c8
            Est,     x = ( 2 1)
            sb: Est
        -----------------------------------------
     */

    return 0;
}
