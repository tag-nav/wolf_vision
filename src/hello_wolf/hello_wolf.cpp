/**
 * \file hello_wolf.cpp
 *
 *  Created on: Dec 1, 2017
 *      \author: jsola
 */


#include "wolf.h"
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

    // sensor
    IntrinsicsRangeBearingPtr intrinsics;
    intrinsics->noise_bearing_degrees_std   = 1.0;
    intrinsics->noise_range_metres_std      = 0.1;
    SensorBasePtr sensor        = problem->installSensor("RANGE BEARING", "sensor", Vector3s(0,0,0), intrinsics);

    // processor
    ProcessorParamsRangeBearingPtr params;
    ProcessorBasePtr processor  = problem->installProcessor("RANGE BEARING", "processor", sensor, params);

    return 0;
}
