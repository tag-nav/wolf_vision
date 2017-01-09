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

    std::cout << "start" << std::endl;
    // Wolf problem
    wolf::ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_3D);
    std::cout << "started" << std::endl;
    //just to make it work
    Eigen::VectorXs extrinsics(7);
    extrinsics << 0,0,0, 0,0,0,1; // pose in the robot
    std::cout << "sensor installing" << std::endl;
    SensorBasePtr sensor_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "Main ODOM_3D", extrinsics, shared_ptr<IntrinsicsOdom3D>());
    std::cout << "sensor installed" << std::endl;
    wolf_problem_ptr_->installProcessor("ODOM 3D", "ODOM_3D integrator", "Main ODOM_3D", "");

    std::cout << "debug" << std::endl;
    ProcessorOdom3D prc;
    Motion ref(0,7,6), final(0,7,6);
    Eigen::VectorXs origin_state(7);
    TimeStamp t_o, ts;

    // set ref
    ref.ts_ = 0;
    ref.delta_          << 0,0,0, 0,0,0,1;
    ref.delta_integr_   << 0,0,0, 0,0,0,1;

    // set final
    final.ts_ = 10;
    final.delta_        << 1,5,0.5, 0,0,0,1;
    final.delta_integr_ << 0,0,0, 0,0,0,1;

    //set origin state
    origin_state << 0,0,0, 0,0,0,1;
    t_o.set(0);

    std::cout << "debug" << std::endl;
    prc.setOrigin(origin_state, t_o);
    std::cout << "debug" << std::endl;
    prc.deltaPlusDelta(ref.delta_integr_, final.delta_, (final.ts_ - ref.ts_), final.delta_integr_);

    //create a keyframe at origin
    StateBlockPtr o_p = std::make_shared<StateBlock>(origin_state.head<3>());
    StateBlockPtr o_q = std::make_shared<StateBlock>(origin_state.tail<3>());
    wolf::FrameBasePtr origin_frame = std::make_shared<FrameBase>(t_o, o_p, o_q);
    wolf_problem_ptr_->getTrajectoryPtr()->addFrame(origin_frame);
    //create a keyframe at final state
    ts = wolf_problem_ptr_->getProcessorMotionPtr()->getBuffer().get().back().ts_;
    Eigen::VectorXs final_state;
    final_state = prc.getCurrentState().head(7);
    StateBlockPtr f_p = std::make_shared<StateBlock>(final_state.head<3>());
    StateBlockPtr f_q = std::make_shared<StateBlock>(final_state.tail<3>());
    wolf::FrameBasePtr final_frame = std::make_shared<FrameBase>(KEY_FRAME, ts, f_p, f_q);
    wolf_problem_ptr_->getTrajectoryPtr()->addFrame(final_frame);
    
    //create a feature
    //FeatureBasePtr last_feature = std::make_shared<FeatureBase>("ODOM_3D", x0.head(7),Eigen::Matrix7s::Identity());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}