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
#include "../constraint_quaternion_absolute.h"
#include "../solver/solver_manager.h"
#include "../ceres_wrapper/ceres_manager.h"
#include "../local_parametrization_angle.h"
#include "../local_parametrization_quaternion.h"

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

        int numConstraints()
        {
            return ceres_problem_->NumResidualBlocks();
        };

        bool isConstraintRegistered(const ConstraintBasePtr& ctr_ptr) const
        {
            return ctr_2_residual_idx_.find(ctr_ptr) != ctr_2_residual_idx_.end() && ctr_2_costfunction_.find(ctr_ptr) != ctr_2_costfunction_.end();
        };

        bool hasThisLocalParametrization(const StateBlockPtr& st, const LocalParametrizationBasePtr& local_param)
        {
            return state_blocks_local_param_.find(st) != state_blocks_local_param_.end() &&
                   state_blocks_local_param_.at(st)->getLocalParametrizationPtr() == local_param &&
                   ceres_problem_->GetParameterization(getAssociatedMemBlockPtr(st)) == state_blocks_local_param_.at(st).get();
        };

        bool hasLocalParametrization(const StateBlockPtr& st) const
        {
            return state_blocks_local_param_.find(st) != state_blocks_local_param_.end();
        };

};

TEST(CeresManager, Create)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // check double ointers to branches
    ASSERT_EQ(P, ceres_manager_ptr->getProblemPtr());

    // run ceres manager check
    ceres_manager_ptr->check();
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

    // run ceres manager check
    ceres_manager_ptr->check();
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

    // run ceres manager check
    ceres_manager_ptr->check();
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

    // run ceres manager check
    ceres_manager_ptr->check();
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

    // run ceres manager check
    ceres_manager_ptr->check();
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
    P->removeStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check stateblock
    ASSERT_FALSE(ceres_manager_ptr->isStateBlockRegisteredSolverManager(sb_ptr));
    ASSERT_EQ(ceres_manager_ptr->numStateBlocks(), 0);

    // run ceres manager check
    ceres_manager_ptr->check();
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
    P->removeStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check no stateblocks
    ASSERT_FALSE(ceres_manager_ptr->isStateBlockRegisteredSolverManager(sb_ptr));
    ASSERT_EQ(ceres_manager_ptr->numStateBlocks(), 0);

    // run ceres manager check
    ceres_manager_ptr->check();
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
    P->removeStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // run ceres manager check
    ceres_manager_ptr->check();
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
    P->removeStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // remove state_block
    P->removeStateBlock(sb_ptr);

    // update solver manager
    ceres_manager_ptr->update();

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, AddConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    // update solver
    ceres_manager_ptr->update();

    // check constraint
    ASSERT_TRUE(ceres_manager_ptr->isConstraintRegistered(c));
    ASSERT_EQ(ceres_manager_ptr->numConstraints(), 1);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, DoubleAddConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    // add constraint again
    P->addConstraint(c);

    // update solver
    ceres_manager_ptr->update();

    // check constraint
    ASSERT_TRUE(ceres_manager_ptr->isConstraintRegistered(c));
    ASSERT_EQ(ceres_manager_ptr->numConstraints(), 1);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, RemoveConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    // update solver
    ceres_manager_ptr->update();

    // remove constraint
    P->removeConstraint(c);

    // update solver
    ceres_manager_ptr->update();

    // check constraint
    ASSERT_FALSE(ceres_manager_ptr->isConstraintRegistered(c));
    ASSERT_EQ(ceres_manager_ptr->numConstraints(), 0);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, AddRemoveConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    ASSERT_TRUE(P->getConstraintNotificationMap().begin()->first == c);

    // remove constraint
    P->removeConstraint(c);

    ASSERT_TRUE(P->getConstraintNotificationMap().empty());

    // update solver
    ceres_manager_ptr->update();

    // check constraint
    ASSERT_FALSE(ceres_manager_ptr->isConstraintRegistered(c));
    ASSERT_EQ(ceres_manager_ptr->numConstraints(), 0);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, DoubleRemoveConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    // update solver
    ceres_manager_ptr->update();

    // remove constraint
    P->removeConstraint(c);

    // update solver
    ceres_manager_ptr->update();

    // remove constraint
    P->removeConstraint(c);

    ASSERT_DEATH({
    // update solver
    ceres_manager_ptr->update();},"");

    // check constraint
    ASSERT_FALSE(ceres_manager_ptr->isConstraintRegistered(c));
    ASSERT_EQ(ceres_manager_ptr->numConstraints(), 0);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, AddStateBlockLocalParam)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector1s state; state << 1;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Local param
    LocalParametrizationBasePtr local_param_ptr = std::make_shared<LocalParametrizationAngle>();
    sb_ptr->setLocalParametrizationPtr(local_param_ptr);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check stateblock
    ASSERT_TRUE(ceres_manager_ptr->hasLocalParametrization(sb_ptr));
    ASSERT_TRUE(ceres_manager_ptr->hasThisLocalParametrization(sb_ptr,local_param_ptr));

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, RemoveLocalParam)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector1s state; state << 1;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Local param
    LocalParametrizationBasePtr local_param_ptr = std::make_shared<LocalParametrizationAngle>();
    sb_ptr->setLocalParametrizationPtr(local_param_ptr);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // Remove local param
    sb_ptr->removeLocalParametrization();

    // update solver
    ceres_manager_ptr->update();

    // check stateblock
    ASSERT_FALSE(ceres_manager_ptr->hasLocalParametrization(sb_ptr));

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, AddLocalParam)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector1s state; state << 1;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check stateblock
    ASSERT_FALSE(ceres_manager_ptr->hasLocalParametrization(sb_ptr));

    // Local param
    LocalParametrizationBasePtr local_param_ptr = std::make_shared<LocalParametrizationAngle>();
    sb_ptr->setLocalParametrizationPtr(local_param_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check stateblock
    ASSERT_TRUE(ceres_manager_ptr->hasLocalParametrization(sb_ptr));
    ASSERT_TRUE(ceres_manager_ptr->hasThisLocalParametrization(sb_ptr,local_param_ptr));

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, ConstraintsRemoveLocalParam)
{
    ProblemPtr P = Problem::create("PO 3D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) 2 constraints quaternion
    FrameBasePtr                    F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr                  C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr                  f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintQuaternionAbsolutePtr c1 = std::static_pointer_cast<ConstraintQuaternionAbsolute>(f->addConstraint(std::make_shared<ConstraintQuaternionAbsolute>(F->getOPtr())));
    ConstraintQuaternionAbsolutePtr c2 = std::static_pointer_cast<ConstraintQuaternionAbsolute>(f->addConstraint(std::make_shared<ConstraintQuaternionAbsolute>(F->getOPtr())));

    // update solver
    ceres_manager_ptr->update();

    // check local param
    ASSERT_TRUE(ceres_manager_ptr->hasLocalParametrization(F->getOPtr()));
    ASSERT_TRUE(ceres_manager_ptr->hasThisLocalParametrization(F->getOPtr(),F->getOPtr()->getLocalParametrizationPtr()));

    // check constraint
    ASSERT_TRUE(ceres_manager_ptr->isConstraintRegistered(c1));
    ASSERT_TRUE(ceres_manager_ptr->isConstraintRegistered(c2));
    ASSERT_EQ(ceres_manager_ptr->numConstraints(), 2);

    // remove local param
    F->getOPtr()->removeLocalParametrization();

    // update solver
    ceres_manager_ptr->update();

    // check local param
    ASSERT_FALSE(ceres_manager_ptr->hasLocalParametrization(F->getOPtr()));

    // check constraint
    ASSERT_TRUE(ceres_manager_ptr->isConstraintRegistered(c1));
    ASSERT_TRUE(ceres_manager_ptr->isConstraintRegistered(c2));
    ASSERT_EQ(ceres_manager_ptr->numConstraints(), 2);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, ConstraintsUpdateLocalParam)
{
    ProblemPtr P = Problem::create("PO 3D");
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) 2 constraints quaternion
    FrameBasePtr                    F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr                  C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr                  f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintQuaternionAbsolutePtr c1 = std::static_pointer_cast<ConstraintQuaternionAbsolute>(f->addConstraint(std::make_shared<ConstraintQuaternionAbsolute>(F->getOPtr())));
    ConstraintQuaternionAbsolutePtr c2 = std::static_pointer_cast<ConstraintQuaternionAbsolute>(f->addConstraint(std::make_shared<ConstraintQuaternionAbsolute>(F->getOPtr())));

    // update solver
    ceres_manager_ptr->update();

    // check local param
    ASSERT_TRUE(ceres_manager_ptr->hasLocalParametrization(F->getOPtr()));
    ASSERT_TRUE(ceres_manager_ptr->hasThisLocalParametrization(F->getOPtr(),F->getOPtr()->getLocalParametrizationPtr()));

    // check constraint
    ASSERT_TRUE(ceres_manager_ptr->isConstraintRegistered(c1));
    ASSERT_TRUE(ceres_manager_ptr->isConstraintRegistered(c2));
    ASSERT_EQ(ceres_manager_ptr->numConstraints(), 2);

    // remove local param
    LocalParametrizationBasePtr local_param_ptr = std::make_shared<LocalParametrizationQuaternionGlobal>();
    F->getOPtr()->setLocalParametrizationPtr(local_param_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check local param
    ASSERT_TRUE(ceres_manager_ptr->hasLocalParametrization(F->getOPtr()));
    ASSERT_TRUE(ceres_manager_ptr->hasThisLocalParametrization(F->getOPtr(),local_param_ptr));

    // check constraint
    ASSERT_TRUE(ceres_manager_ptr->isConstraintRegistered(c1));
    ASSERT_TRUE(ceres_manager_ptr->isConstraintRegistered(c2));
    ASSERT_EQ(ceres_manager_ptr->numConstraints(), 2);

    // run ceres manager check
    ceres_manager_ptr->check();
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

