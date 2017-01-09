/**
 * \file gtest_constraint_odom_3D.cpp
 *
 *  Created on: Jan 06, 2016
 *      \author: Dinesh
 */

#include "wolf.h"
#include "problem.h"
#include "sensor_odom_3D.h"
#include "capture_motion.h"
#include "state_block.h"
#include "state_quaternion.h"
#include "processor_odom_3D.h"

#include "utils_gtest.h"
#include "../src/logging.h"

TEST(ConstraintOdom3D, constructors)
{
    using namespace wolf;
    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    // Wolf problem
    wolf::ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs IMU_extrinsics(7);

    wolf::TimeStamp ts;
    Eigen::VectorXs state_vec;
    Eigen::VectorXs delta_preint;
    Eigen::Vector6s data_;

    state_vec.resize(16);
    ts.set(0);

    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0; // Try some non-zero biases

    //create a feature
    FeatureBasePtr last_feature = std::make_shared<FeatureBase>("ODOM_3D", x0.head(7),Eigen::Matrix7s::Identity());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}