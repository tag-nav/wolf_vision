/*
 * gtest_ceres_manager.cpp
 *
 *  Created on: Jun, 2018
 *      Author: jvallve
 */

#include "utils_gtest.h"
#include "../src/logging.h"

#include "../problem.h"
#include "../sensor_base.h"
#include "../state_block.h"
#include "../capture_void.h"
#include "../constraint_pose_2D.h"
#include "../solver/solver_manager.h"
#include "../ceres_wrapper/ceres_manager.h"

#include "ceres/ceres.h"

#include <iostream>

using namespace wolf;
using namespace Eigen;

WOLF_PTR_TYPEDEFS(CeresManagerWrapper);
class CeresManagerWrapper : public CeresManager
{
    public:
        CeresManagerWrapper(const ProblemPtr& wolf_problem) :
            CeresManager(wolf_problem)
        {
        };

        bool isStateBlockRegisteredCeresManager(const StateBlockPtr& st)
        {
            return ceres_problem_->HasParameterBlock(SolverManager::getAssociatedMemBlockPtr(st));
        };

        bool isStateBlockRegisteredSolverManager(const StateBlockPtr& st)
        {
            return state_blocks_.find(st)!=state_blocks_.end();
        };

        bool isStateBlockFixed(const StateBlockPtr& st)
        {
            return ceres_problem_->IsParameterBlockConstant(SolverManager::getAssociatedMemBlockPtr(st));
        };

        int numStateBlocks()
        {
            return ceres_problem_->NumParameterBlocks();
        };

        bool isConstraintRegistered(const ConstraintBasePtr& ctr_ptr) const
        {
            return ctr_2_residual_idx_.find(ctr_ptr) != ctr_2_residual_idx_.end() && ctr_2_costfunction_.find(ctr_ptr) != ctr_2_costfunction_.end();
        };

};

TEST(CeresManager, Create)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // check double ointers to branches
    ASSERT_EQ(P, ceres_manager_ptr->getProblemPtr());
}

TEST(CeresManager, AddStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check stateblock
    ASSERT_TRUE(ceres_manager_ptr->isStateBlockRegisteredSolverManager(sb_ptr));
    ASSERT_TRUE(ceres_manager_ptr->isStateBlockRegisteredCeresManager(sb_ptr));
}

TEST(CeresManager, DoubleAddStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // add stateblock again
    P->addStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check stateblock
    ASSERT_TRUE(ceres_manager_ptr->isStateBlockRegisteredSolverManager(sb_ptr));
    ASSERT_TRUE(ceres_manager_ptr->isStateBlockRegisteredCeresManager(sb_ptr));
}

TEST(CeresManager, UpdateStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check stateblock unfixed
    ASSERT_FALSE(ceres_manager_ptr->isStateBlockFixed(sb_ptr));

    // Fix frame
    sb_ptr->fix();

    // update solver manager
    ceres_manager_ptr->update();

    // check stateblock fixed
    ASSERT_TRUE(ceres_manager_ptr->isStateBlockFixed(sb_ptr));
}

TEST(CeresManager, AddUpdateStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // Fix state block
    sb_ptr->fix();

    // update solver manager
    ceres_manager_ptr->update();

    // check stateblock fixed
    ASSERT_TRUE(ceres_manager_ptr->isStateBlockRegisteredSolverManager(sb_ptr));
    ASSERT_TRUE(ceres_manager_ptr->isStateBlockRegisteredCeresManager(sb_ptr));
    ASSERT_TRUE(ceres_manager_ptr->isStateBlockFixed(sb_ptr));
}

TEST(CeresManager, RemoveStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    ASSERT_TRUE(ceres_manager_ptr->isStateBlockRegisteredSolverManager(sb_ptr));
    ASSERT_TRUE(ceres_manager_ptr->isStateBlockRegisteredCeresManager(sb_ptr));

    // remove state_block
    P->removeStateBlockPtr(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check stateblock
    ASSERT_FALSE(ceres_manager_ptr->isStateBlockRegisteredSolverManager(sb_ptr));
    ASSERT_TRUE(ceres_manager_ptr->numStateBlocks() == 0);
}

TEST(CeresManager, AddRemoveStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // remove state_block
    P->removeStateBlockPtr(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check no stateblocks
    ASSERT_FALSE(ceres_manager_ptr->isStateBlockRegisteredSolverManager(sb_ptr));
    ASSERT_TRUE(ceres_manager_ptr->numStateBlocks() == 0);
}

TEST(CeresManager, RemoveUpdateStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // remove state_block
    P->removeStateBlockPtr(sb_ptr);

    // update solver
    ceres_manager_ptr->update();
}

TEST(CeresManager, DoubleRemoveStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // remove state_block
    P->removeStateBlockPtr(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // remove state_block
    P->removeStateBlockPtr(sb_ptr);

    // update solver manager
    ceres_manager_ptr->update();
}

TEST(CeresManager, AddConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    // update solver
    ceres_manager_ptr->update();

    // check constraint
    ASSERT_TRUE(ceres_manager_ptr->isConstraintRegistered(c));
}

TEST(CeresManager, RemoveConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    // update solver
    ceres_manager_ptr->update();

    // add constraint
    P->removeConstraintPtr(c);

    // update solver
    ceres_manager_ptr->update();

    // check constraint
    ASSERT_FALSE(ceres_manager_ptr->isConstraintRegistered(c));
}

TEST(CeresManager, AddRemoveConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    ASSERT_TRUE(P->getConstraintNotificationList().front().constraint_ptr_ == c);

    // add constraint
    P->removeConstraintPtr(c);

    ASSERT_TRUE(P->getConstraintNotificationList().empty());

    // update solver
    ceres_manager_ptr->update();

    // check constraint
    ASSERT_FALSE(ceres_manager_ptr->isConstraintRegistered(c));
}

TEST(CeresManager, DoubleRemoveConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    // update solver
    ceres_manager_ptr->update();

    // remove constraint
    P->removeConstraintPtr(c);

    // update solver
    ceres_manager_ptr->update();

    // remove constraint
    P->removeConstraintPtr(c);

    ASSERT_DEATH({
    // update solver
    ceres_manager_ptr->update();},"");

    // check constraint
    ASSERT_FALSE(ceres_manager_ptr->isConstraintRegistered(c));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

