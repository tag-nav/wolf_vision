/*
 * gtest_ceres_manager.cpp
 *
 *  Created on: Jun, 2018
 *      Author: jvallve
 */

#include "utils_gtest.h"
#include "core/utils/logging.h"

#include "core/problem/problem.h"
#include "core/sensor/sensor_base.h"
#include "core/state_block/state_block.h"
#include "core/capture/capture_void.h"
#include "core/factor/factor_pose_2D.h"
#include "core/factor/factor_quaternion_absolute.h"
#include "core/solver/solver_manager.h"
#include "core/ceres_wrapper/ceres_manager.h"
#include "core/state_block/local_parametrization_angle.h"
#include "core/state_block/local_parametrization_quaternion.h"

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

        int numFactors()
        {
            return ceres_problem_->NumResidualBlocks();
        };

        bool isFactorRegistered(const FactorBasePtr& fac_ptr) const
        {
            return fac_2_residual_idx_.find(fac_ptr) != fac_2_residual_idx_.end() && fac_2_costfunction_.find(fac_ptr) != fac_2_costfunction_.end();
        };

        bool hasThisLocalParametrization(const StateBlockPtr& st, const LocalParametrizationBasePtr& local_param)
        {
            return state_blocks_local_param_.find(st) != state_blocks_local_param_.end() &&
                   state_blocks_local_param_.at(st)->getLocalParametrization() == local_param &&
                   ceres_problem_->GetParameterization(getAssociatedMemBlockPtr(st)) == state_blocks_local_param_.at(st).get();
        };

        bool hasLocalParametrization(const StateBlockPtr& st) const
        {
            return state_blocks_local_param_.find(st) != state_blocks_local_param_.end();
        };

};

TEST(CeresManager, Create)
{
    ProblemPtr P = Problem::create("PO", 2);
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // check double ointers to branches
    ASSERT_EQ(P, ceres_manager_ptr->getProblem());

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, AddStateBlock)
{
    ProblemPtr P = Problem::create("PO", 2);
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
    ProblemPtr P = Problem::create("PO", 2);
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
    ProblemPtr P = Problem::create("PO", 2);
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
    ProblemPtr P = Problem::create("PO", 2);
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
    ProblemPtr P = Problem::create("PO", 2);
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
    ProblemPtr P = Problem::create("PO", 2);
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
    ProblemPtr P = Problem::create("PO", 2);
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
    ProblemPtr P = Problem::create("PO", 2);
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

TEST(CeresManager, AddFactor)
{
    ProblemPtr P = Problem::create("PO", 2);
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) factor point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY, P->zeroState(), TimeStamp(0));
    auto C = CaptureBase::emplace<CaptureVoid>(F, 0, nullptr);
    auto f = FeatureBase::emplace<FeatureBase>(C, "ODOM 2D", Vector3s::Zero(), Matrix3s::Identity());
    FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(FactorBase::emplace<FactorPose2D>(f,f));

    // update solver
    ceres_manager_ptr->update();

    // check factor
    ASSERT_TRUE(ceres_manager_ptr->isFactorRegistered(c));
    ASSERT_EQ(ceres_manager_ptr->numFactors(), 1);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, DoubleAddFactor)
{
    ProblemPtr P = Problem::create("PO", 2);
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) factor point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY, P->zeroState(), TimeStamp(0));
    auto C = CaptureBase::emplace<CaptureVoid>(F, 0, nullptr);
    auto f = FeatureBase::emplace<FeatureBase>(C, "ODOM 2D", Vector3s::Zero(), Matrix3s::Identity());
    FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(FactorBase::emplace<FactorPose2D>(f,f));

    // add factor again
    P->addFactor(c);

    // update solver
    ceres_manager_ptr->update();

    // check factor
    ASSERT_TRUE(ceres_manager_ptr->isFactorRegistered(c));
    ASSERT_EQ(ceres_manager_ptr->numFactors(), 1);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, RemoveFactor)
{
    ProblemPtr P = Problem::create("PO", 2);
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) factor point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY, P->zeroState(), TimeStamp(0));
    auto C = CaptureBase::emplace<CaptureVoid>(F, 0, nullptr);
    auto f = FeatureBase::emplace<FeatureBase>(C, "ODOM 2D", Vector3s::Zero(), Matrix3s::Identity());
    FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(FactorBase::emplace<FactorPose2D>(f,f));

    // update solver
    ceres_manager_ptr->update();

    // remove factor
    P->removeFactor(c);

    // update solver
    ceres_manager_ptr->update();

    // check factor
    ASSERT_FALSE(ceres_manager_ptr->isFactorRegistered(c));
    ASSERT_EQ(ceres_manager_ptr->numFactors(), 0);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, AddRemoveFactor)
{
    ProblemPtr P = Problem::create("PO", 2);
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) factor point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY, P->zeroState(), TimeStamp(0));
    auto C = CaptureBase::emplace<CaptureVoid>(F, 0, nullptr);
    auto f = FeatureBase::emplace<FeatureBase>(C, "ODOM 2D", Vector3s::Zero(), Matrix3s::Identity());
    FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(FactorBase::emplace<FactorPose2D>(f,f));

    // remove factor
    P->removeFactor(c);

    ASSERT_TRUE(P->getFactorNotificationMapSize() == 0); // add+remove = empty

    // update solver
    ceres_manager_ptr->update();

    // check factor
    ASSERT_FALSE(ceres_manager_ptr->isFactorRegistered(c));
    ASSERT_EQ(ceres_manager_ptr->numFactors(), 0);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, DoubleRemoveFactor)
{
    ProblemPtr P = Problem::create("PO", 2);
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) factor point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY, P->zeroState(), TimeStamp(0));
    auto C = CaptureBase::emplace<CaptureVoid>(F, 0, nullptr);
    auto f = FeatureBase::emplace<FeatureBase>(C, "ODOM 2D", Vector3s::Zero(), Matrix3s::Identity());
    FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(FactorBase::emplace<FactorPose2D>(f,f));

    // update solver
    ceres_manager_ptr->update();

    // remove factor
    P->removeFactor(c);

    // update solver
    ceres_manager_ptr->update();

    // remove factor
    P->removeFactor(c);

    ASSERT_DEATH({
    // update solver
    ceres_manager_ptr->update();},"");

    // check factor
    ASSERT_FALSE(ceres_manager_ptr->isFactorRegistered(c));
    ASSERT_EQ(ceres_manager_ptr->numFactors(), 0);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, AddStateBlockLocalParam)
{
    ProblemPtr P = Problem::create("PO", 2);
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector1s state; state << 1;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Local param
    LocalParametrizationBasePtr local_param_ptr = std::make_shared<LocalParametrizationAngle>();
    sb_ptr->setLocalParametrization(local_param_ptr);

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
    ProblemPtr P = Problem::create("PO", 2);
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create State block
    Vector1s state; state << 1;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Local param
    LocalParametrizationBasePtr local_param_ptr = std::make_shared<LocalParametrizationAngle>();
    sb_ptr->setLocalParametrization(local_param_ptr);

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
    ProblemPtr P = Problem::create("PO", 2);
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
    sb_ptr->setLocalParametrization(local_param_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check stateblock
    ASSERT_TRUE(ceres_manager_ptr->hasLocalParametrization(sb_ptr));
    ASSERT_TRUE(ceres_manager_ptr->hasThisLocalParametrization(sb_ptr,local_param_ptr));

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, FactorsRemoveLocalParam)
{
    ProblemPtr P = Problem::create("PO", 3);
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) 2 factors quaternion
    FrameBasePtr                    F = P->emplaceFrame(KEY, P->zeroState(), TimeStamp(0));
    auto C = CaptureBase::emplace<CaptureVoid>(F, 0, nullptr);
    auto f = FeatureBase::emplace<FeatureBase>(C, "ODOM 2D", Vector3s::Zero(), Matrix3s::Identity());
    FactorQuaternionAbsolutePtr c1 = std::static_pointer_cast<FactorQuaternionAbsolute>(FactorBase::emplace<FactorQuaternionAbsolute>(f, F->getO()));
    FactorQuaternionAbsolutePtr c2 = std::static_pointer_cast<FactorQuaternionAbsolute>(FactorBase::emplace<FactorQuaternionAbsolute>(f, F->getO()));

    // update solver
    ceres_manager_ptr->update();

    // check local param
    ASSERT_TRUE(ceres_manager_ptr->hasLocalParametrization(F->getO()));
    ASSERT_TRUE(ceres_manager_ptr->hasThisLocalParametrization(F->getO(),F->getO()->getLocalParametrization()));

    // check factor
    ASSERT_TRUE(ceres_manager_ptr->isFactorRegistered(c1));
    ASSERT_TRUE(ceres_manager_ptr->isFactorRegistered(c2));
    ASSERT_EQ(ceres_manager_ptr->numFactors(), 2);

    // remove local param
    F->getO()->removeLocalParametrization();

    // update solver
    ceres_manager_ptr->update();

    // check local param
    ASSERT_FALSE(ceres_manager_ptr->hasLocalParametrization(F->getO()));

    // check factor
    ASSERT_TRUE(ceres_manager_ptr->isFactorRegistered(c1));
    ASSERT_TRUE(ceres_manager_ptr->isFactorRegistered(c2));
    ASSERT_EQ(ceres_manager_ptr->numFactors(), 2);

    // run ceres manager check
    ceres_manager_ptr->check();
}

TEST(CeresManager, FactorsUpdateLocalParam)
{
    ProblemPtr P = Problem::create("PO", 3);
    CeresManagerWrapperPtr ceres_manager_ptr = std::make_shared<CeresManagerWrapper>(P);

    // Create (and add) 2 factors quaternion
    FrameBasePtr                    F = P->emplaceFrame(KEY, P->zeroState(), TimeStamp(0));
    auto C = CaptureBase::emplace<CaptureVoid>(F, 0, nullptr);
    auto f = FeatureBase::emplace<FeatureBase>(C, "ODOM 2D", Vector3s::Zero(), Matrix3s::Identity());
    FactorQuaternionAbsolutePtr c1 = std::static_pointer_cast<FactorQuaternionAbsolute>(FactorBase::emplace<FactorQuaternionAbsolute>(f, F->getO()));
    FactorQuaternionAbsolutePtr c2 = std::static_pointer_cast<FactorQuaternionAbsolute>(FactorBase::emplace<FactorQuaternionAbsolute>(f, F->getO()));

    // update solver
    ceres_manager_ptr->update();

    // check local param
    ASSERT_TRUE(ceres_manager_ptr->hasLocalParametrization(F->getO()));
    ASSERT_TRUE(ceres_manager_ptr->hasThisLocalParametrization(F->getO(),F->getO()->getLocalParametrization()));

    // check factor
    ASSERT_TRUE(ceres_manager_ptr->isFactorRegistered(c1));
    ASSERT_TRUE(ceres_manager_ptr->isFactorRegistered(c2));
    ASSERT_EQ(ceres_manager_ptr->numFactors(), 2);

    // remove local param
    LocalParametrizationBasePtr local_param_ptr = std::make_shared<LocalParametrizationQuaternionGlobal>();
    F->getO()->setLocalParametrization(local_param_ptr);

    // update solver
    ceres_manager_ptr->update();

    // check local param
    ASSERT_TRUE(ceres_manager_ptr->hasLocalParametrization(F->getO()));
    ASSERT_TRUE(ceres_manager_ptr->hasThisLocalParametrization(F->getO(),local_param_ptr));

    // check factor
    ASSERT_TRUE(ceres_manager_ptr->isFactorRegistered(c1));
    ASSERT_TRUE(ceres_manager_ptr->isFactorRegistered(c2));
    ASSERT_EQ(ceres_manager_ptr->numFactors(), 2);

    // run ceres manager check
    ceres_manager_ptr->check();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

