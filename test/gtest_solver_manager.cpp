/*
 * gtest_solver_manager.cpp
 *
 *  Created on: Jun, 2018
 *      Author: jvallve
 */

#include "utils_gtest.h"
#include "base/utils/logging.h"

#include "base/problem/problem.h"
#include "base/sensor/sensor_base.h"
#include "base/state_block/state_block.h"
#include "base/capture/capture_void.h"
#include "base/factor/factor_pose_2D.h"
#include "base/solver/solver_manager.h"
#include "base/state_block/local_parametrization_base.h"
#include "base/state_block/local_parametrization_angle.h"

#include <iostream>

using namespace wolf;
using namespace Eigen;

WOLF_PTR_TYPEDEFS(SolverManagerWrapper);
class SolverManagerWrapper : public SolverManager
{
    public:
        std::list<FactorBasePtr> factors_;
        std::map<StateBlockPtr,bool> state_block_fixed_;
        std::map<StateBlockPtr,LocalParametrizationBasePtr> state_block_local_param_;

        SolverManagerWrapper(const ProblemPtr& wolf_problem) :
            SolverManager(wolf_problem)
        {
        };

        bool isStateBlockRegistered(const StateBlockPtr& st) const
        {
            return state_blocks_.find(st)!=state_blocks_.end();
        };

        bool isStateBlockFixed(const StateBlockPtr& st) const
        {
            return state_block_fixed_.at(st);
        };

        bool isFactorRegistered(const FactorBasePtr& fac_ptr) const
        {
            return std::find(factors_.begin(), factors_.end(), fac_ptr) != factors_.end();
        };

        bool hasThisLocalParametrization(const StateBlockPtr& st, const LocalParametrizationBasePtr& local_param) const
        {
            return state_block_local_param_.find(st) != state_block_local_param_.end() && state_block_local_param_.at(st) == local_param;
        };

        bool hasLocalParametrization(const StateBlockPtr& st) const
        {
            return state_block_local_param_.find(st) != state_block_local_param_.end();
        };

        virtual void computeCovariances(const CovarianceBlocksToBeComputed blocks){};
        virtual void computeCovariances(const StateBlockPtrList& st_list){};

        // The following are dummy implementations
        bool    hasConverged()  { return true;      }
        SizeStd iterations()    { return 1;         }
        Scalar  initialCost()   { return Scalar(1); }
        Scalar  finalCost()     { return Scalar(0); }



    protected:

        virtual std::string solveImpl(const ReportVerbosity report_level){ return std::string("");};
        virtual void addFactor(const FactorBasePtr& fac_ptr)
        {
            factors_.push_back(fac_ptr);
        };
        virtual void removeFactor(const FactorBasePtr& fac_ptr)
        {
            factors_.remove(fac_ptr);
        };
        virtual void addStateBlock(const StateBlockPtr& state_ptr)
        {
            state_block_fixed_[state_ptr] = state_ptr->isFixed();
            state_block_local_param_[state_ptr] = state_ptr->getLocalParametrization();
        };
        virtual void removeStateBlock(const StateBlockPtr& state_ptr)
        {
            state_block_fixed_.erase(state_ptr);
            state_block_local_param_.erase(state_ptr);
        };
        virtual void updateStateBlockStatus(const StateBlockPtr& state_ptr)
        {
            state_block_fixed_[state_ptr] = state_ptr->isFixed();
        };
        virtual void updateStateBlockLocalParametrization(const StateBlockPtr& state_ptr)
        {
            if (state_ptr->getLocalParametrization() == nullptr)
                state_block_local_param_.erase(state_ptr);
            else
                state_block_local_param_[state_ptr] = state_ptr->getLocalParametrization();
        };
};

TEST(SolverManager, Create)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // check double pointers to branches
    ASSERT_EQ(P, solver_manager_ptr->getProblem());
}

TEST(SolverManager, AddStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    // check stateblock
    ASSERT_TRUE(solver_manager_ptr->isStateBlockRegistered(sb_ptr));
}

TEST(SolverManager, DoubleAddStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    // add stateblock again
    P->addStateBlock(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    // check stateblock
    ASSERT_TRUE(solver_manager_ptr->isStateBlockRegistered(sb_ptr));
}

TEST(SolverManager, UpdateStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_FALSE(sb_ptr->fixUpdated());
    ASSERT_FALSE(sb_ptr->localParamUpdated());

    // update solver
    solver_manager_ptr->update();

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_FALSE(sb_ptr->fixUpdated());
    ASSERT_FALSE(sb_ptr->localParamUpdated());

    // check stateblock unfixed
    ASSERT_FALSE(solver_manager_ptr->isStateBlockFixed(sb_ptr));

    // Fix frame
    sb_ptr->fix();

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_TRUE(sb_ptr->fixUpdated());
    ASSERT_FALSE(sb_ptr->localParamUpdated());

    // update solver manager
    solver_manager_ptr->update();

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_FALSE(sb_ptr->fixUpdated());
    ASSERT_FALSE(sb_ptr->localParamUpdated());

    // check stateblock fixed
    ASSERT_TRUE(solver_manager_ptr->isStateBlockFixed(sb_ptr));
}

TEST(SolverManager, AddUpdateStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_FALSE(sb_ptr->fixUpdated());
    ASSERT_FALSE(sb_ptr->localParamUpdated());

    // Fix state block
    sb_ptr->fix();

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_TRUE(sb_ptr->fixUpdated());
    ASSERT_FALSE(sb_ptr->localParamUpdated());

    // update solver manager
    solver_manager_ptr->update();

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_FALSE(sb_ptr->fixUpdated());
    ASSERT_FALSE(sb_ptr->localParamUpdated());

    // check stateblock fixed
    ASSERT_TRUE(solver_manager_ptr->isStateBlockRegistered(sb_ptr));
    ASSERT_TRUE(solver_manager_ptr->isStateBlockFixed(sb_ptr));
}

TEST(SolverManager, AddUpdateLocalParamStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_FALSE(sb_ptr->fixUpdated());
    ASSERT_FALSE(sb_ptr->localParamUpdated());

    // Local param
    LocalParametrizationBasePtr local_ptr = std::make_shared<LocalParametrizationAngle>();
    sb_ptr->setLocalParametrization(local_ptr);

    // Fix state block
    sb_ptr->fix();

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_TRUE(sb_ptr->fixUpdated());
    ASSERT_TRUE(sb_ptr->localParamUpdated());

    // update solver manager
    solver_manager_ptr->update();

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_FALSE(sb_ptr->fixUpdated());
    ASSERT_FALSE(sb_ptr->localParamUpdated());

    // check stateblock fixed
    ASSERT_TRUE(solver_manager_ptr->isStateBlockRegistered(sb_ptr));
    ASSERT_TRUE(solver_manager_ptr->isStateBlockFixed(sb_ptr));
    ASSERT_TRUE(solver_manager_ptr->hasThisLocalParametrization(sb_ptr,local_ptr));
}

TEST(SolverManager, AddLocalParamRemoveLocalParamStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Local param
    LocalParametrizationBasePtr local_ptr = std::make_shared<LocalParametrizationAngle>();
    sb_ptr->setLocalParametrization(local_ptr);

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_FALSE(sb_ptr->fixUpdated());
    ASSERT_TRUE(sb_ptr->localParamUpdated());

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver manager
    solver_manager_ptr->update();

    // check stateblock localparam
    ASSERT_TRUE(solver_manager_ptr->hasThisLocalParametrization(sb_ptr,local_ptr));

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_FALSE(sb_ptr->fixUpdated());
    ASSERT_FALSE(sb_ptr->localParamUpdated());

    // Remove local param
    sb_ptr->removeLocalParametrization();

    // check flags
    ASSERT_FALSE(sb_ptr->stateUpdated());
    ASSERT_FALSE(sb_ptr->fixUpdated());
    ASSERT_TRUE(sb_ptr->localParamUpdated());

    // update solver manager
    solver_manager_ptr->update();

    // check stateblock localparam
    ASSERT_FALSE(solver_manager_ptr->hasLocalParametrization(sb_ptr));
}

TEST(SolverManager, RemoveStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    // remove state_block
    P->removeStateBlock(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    // check stateblock
    ASSERT_FALSE(solver_manager_ptr->isStateBlockRegistered(sb_ptr));
}

TEST(SolverManager, AddRemoveStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // remove state_block
    P->removeStateBlock(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    // check state block
    ASSERT_FALSE(solver_manager_ptr->isStateBlockRegistered(sb_ptr));
}

TEST(SolverManager, RemoveUpdateStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add state_block
    P->addStateBlock(sb_ptr);

    // remove state_block
    P->removeStateBlock(sb_ptr);

    // update solver
    solver_manager_ptr->update();
}

TEST(SolverManager, DoubleRemoveStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // remove state_block
    P->removeStateBlock(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    // remove state_block
    P->removeStateBlock(sb_ptr);

    // update solver manager
    solver_manager_ptr->update();
}

TEST(SolverManager, AddUpdatedStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Fix
    sb_ptr->fix();

    // Set State
    Vector2s state_2 = 2*state;
    sb_ptr->setState(state_2);

    // Check flags have been set true
    ASSERT_TRUE(sb_ptr->fixUpdated());
    ASSERT_TRUE(sb_ptr->stateUpdated());

    // == When an ADD is notified: a ADD notification should be stored ==

    // add state_block
    P->addStateBlock(sb_ptr);

    auto state_block_notification_map = P->consumeStateBlockNotificationMap();
    ASSERT_EQ(state_block_notification_map.size(),1);
    ASSERT_EQ(state_block_notification_map.begin()->second,ADD);

    // == Insert OTHER notifications ==

    // Set State --> FLAG
    state_2 = 2*state;
    sb_ptr->setState(state_2);

    // Fix --> FLAG
    sb_ptr->unfix();

    ASSERT_TRUE(P->consumeStateBlockNotificationMap().empty()); // No new notifications (fix and set state are flags in sb)

    // == When an REMOVE is notified: a REMOVE notification should be stored ==

    // remove state_block
    P->removeStateBlock(sb_ptr);

    state_block_notification_map = P->consumeStateBlockNotificationMap();
    ASSERT_EQ(state_block_notification_map.size(),1);
    ASSERT_EQ(state_block_notification_map.begin()->second,REMOVE);

    // == ADD + REMOVE: notification map should be empty ==
    P->addStateBlock(sb_ptr);
    P->removeStateBlock(sb_ptr);
    ASSERT_TRUE(P->consumeStateBlockNotificationMap().empty());

    // == UPDATES should clear the list of notifications ==
    // add state_block
    P->addStateBlock(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    ASSERT_TRUE(P->consumeStateBlockNotificationMap().empty()); // After solver_manager->update, notifications should be empty
}

TEST(SolverManager, AddFactor)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) factor point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    // CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    auto C = CaptureBase::emplace<CaptureVoid>(F, 0, nullptr);
    // FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    auto f = FeatureBase::emplace<FeatureBase>(C, "ODOM 2D", Vector3s::Zero(), Matrix3s::Identity());
    // FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(f->addFactor(std::make_shared<FactorPose2D>(f)));
    FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(FactorBase::emplace<FactorPose2D>(f, f));

    // update solver
    solver_manager_ptr->update();

    // check factor
    ASSERT_TRUE(solver_manager_ptr->isFactorRegistered(c));
}

TEST(SolverManager, RemoveFactor)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) factor point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    // CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    auto C = CaptureBase::emplace<CaptureVoid>(F, 0, nullptr);
    // FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    auto f = FeatureBase::emplace<FeatureBase>(C, "ODOM 2D", Vector3s::Zero(), Matrix3s::Identity());
    // FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(f->addFactor(std::make_shared<FactorPose2D>(f)));
    FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(FactorBase::emplace<FactorPose2D>(f, f));

    // update solver
    solver_manager_ptr->update();

    // add factor
    P->removeFactor(c);

    // update solver
    solver_manager_ptr->update();

    // check factor
    ASSERT_FALSE(solver_manager_ptr->isFactorRegistered(c));
}

TEST(SolverManager, AddRemoveFactor)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) factor point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    // CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    // FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    // FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(f->addFactor(std::make_shared<FactorPose2D>(f)));

    auto C = CaptureBase::emplace<CaptureVoid>(F, 0, nullptr);
    auto f = FeatureBase::emplace<FeatureBase>(C, "ODOM 2D", Vector3s::Zero(), Matrix3s::Identity());
    FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(FactorBase::emplace<FactorPose2D>(f, f));

    // add factor
    P->removeFactor(c);

    ASSERT_TRUE(P->consumeFactorNotificationMap().empty()); // ADD+REMOVE = empty

    // update solver
    solver_manager_ptr->update();

    // check factor
    ASSERT_FALSE(solver_manager_ptr->isFactorRegistered(c));
}

TEST(SolverManager, DoubleRemoveFactor)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) factor point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    // CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    // FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    // FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(f->addFactor(std::make_shared<FactorPose2D>(f)));

    auto C = CaptureBase::emplace<CaptureVoid>(F, 0, nullptr);
    auto f = FeatureBase::emplace<FeatureBase>(C, "ODOM 2D", Vector3s::Zero(), Matrix3s::Identity());
    FactorPose2DPtr c = std::static_pointer_cast<FactorPose2D>(FactorBase::emplace<FactorPose2D>(f, f));

    // update solver
    solver_manager_ptr->update();

    // remove factor
    P->removeFactor(c);

    // update solver
    solver_manager_ptr->update();

    // remove factor
    P->removeFactor(c);

    // update solver
    solver_manager_ptr->update();

    // check factor
    ASSERT_FALSE(solver_manager_ptr->isFactorRegistered(c));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

