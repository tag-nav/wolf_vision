/*
 * gtest_solver_manager.cpp
 *
 *  Created on: Jun, 2018
 *      Author: jvallve
 */

#include "utils_gtest.h"
#include "base/logging.h"

#include "base/problem.h"
#include "base/sensor/sensor_base.h"
#include "base/state_block.h"
#include "base/capture/capture_void.h"
#include "base/constraint/constraint_pose_2D.h"
#include "base/solver/solver_manager.h"
#include "base/local_parametrization_base.h"
#include "base/local_parametrization_angle.h"

#include <iostream>

using namespace wolf;
using namespace Eigen;

WOLF_PTR_TYPEDEFS(SolverManagerWrapper);
class SolverManagerWrapper : public SolverManager
{
    public:
        std::list<ConstraintBasePtr> constraints_;
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

        bool isConstraintRegistered(const ConstraintBasePtr& ctr_ptr) const
        {
            return std::find(constraints_.begin(), constraints_.end(), ctr_ptr) != constraints_.end();
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
        virtual void computeCovariances(const StateBlockList& st_list){};

        // The following are dummy implementations
        bool    hasConverged()  { return true;      }
        SizeStd iterations()    { return 1;         }
        Scalar  initialCost()   { return Scalar(1); }
        Scalar  finalCost()     { return Scalar(0); }



    protected:

        virtual std::string solveImpl(const ReportVerbosity report_level){ return std::string("");};
        virtual void addConstraint(const ConstraintBasePtr& ctr_ptr)
        {
            constraints_.push_back(ctr_ptr);
        };
        virtual void removeConstraint(const ConstraintBasePtr& ctr_ptr)
        {
            constraints_.remove(ctr_ptr);
        };
        virtual void addStateBlock(const StateBlockPtr& state_ptr)
        {
            state_block_fixed_[state_ptr] = state_ptr->isFixed();
            state_block_local_param_[state_ptr] = state_ptr->getLocalParametrizationPtr();
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
            if (state_ptr->getLocalParametrizationPtr() == nullptr)
                state_block_local_param_.erase(state_ptr);
            else
                state_block_local_param_[state_ptr] = state_ptr->getLocalParametrizationPtr();
        };
};

TEST(SolverManager, Create)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // check double pointers to branches
    ASSERT_EQ(P, solver_manager_ptr->getProblemPtr());
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
    sb_ptr->setLocalParametrizationPtr(local_ptr);

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
    sb_ptr->setLocalParametrizationPtr(local_ptr);

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

    // == When an ADD is notified, all previous notifications should be cleared (if not consumption of notifs) ==

    // add state_block
    P->addStateBlock(sb_ptr);

    ASSERT_EQ(P->getStateBlockNotificationMap().size(),1);
    ASSERT_EQ(P->getStateBlockNotificationMap().begin()->second,ADD);

    // == Insert OTHER notifications ==

    // Set State --> FLAG
    state_2 = 2*state;
    sb_ptr->setState(state_2);

    // Fix --> FLAG
    sb_ptr->unfix();

    ASSERT_EQ(P->getStateBlockNotificationMap().size(),1); // only ADD notification (fix and state are flags in sb)

    // == REMOVE should clear the previous notification (ADD) in the stack ==

    // remove state_block
    P->removeStateBlock(sb_ptr);

    ASSERT_EQ(P->getStateBlockNotificationMap().size(),0);// ADD + REMOVE = EMPTY

    // == UPDATES + REMOVE should clear the list of notifications ==

    // add state_block
    P->addStateBlock(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    ASSERT_EQ(P->getStateBlockNotificationMap().size(),0); // After solver_manager->update, notifications should be empty

    // Fix
    sb_ptr->fix();

    // Set State
    state_2 = 2*state;
    sb_ptr->setState(state_2);

    // remove state_block
    P->removeStateBlock(sb_ptr);

    ASSERT_EQ(P->getStateBlockNotificationMap().size(),1);
    ASSERT_EQ(P->getStateBlockNotificationMap().begin()->second, REMOVE);
}

TEST(SolverManager, AddConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    // update solver
    solver_manager_ptr->update();

    // check constraint
    ASSERT_TRUE(solver_manager_ptr->isConstraintRegistered(c));
}

TEST(SolverManager, RemoveConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    // update solver
    solver_manager_ptr->update();

    // add constraint
    P->removeConstraint(c);

    // update solver
    solver_manager_ptr->update();

    // check constraint
    ASSERT_FALSE(solver_manager_ptr->isConstraintRegistered(c));
}

TEST(SolverManager, AddRemoveConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    ASSERT_TRUE(P->getConstraintNotificationMap().begin()->first == c);

    // add constraint
    P->removeConstraint(c);

    ASSERT_TRUE(P->getConstraintNotificationMap().empty());

    // update solver
    solver_manager_ptr->update();

    // check constraint
    ASSERT_FALSE(solver_manager_ptr->isConstraintRegistered(c));
}

TEST(SolverManager, DoubleRemoveConstraint)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // Create (and add) constraint point 2d
    FrameBasePtr        F = P->emplaceFrame(KEY_FRAME, P->zeroState(), TimeStamp(0));
    CaptureBasePtr      C = F->addCapture(std::make_shared<CaptureVoid>(0, nullptr));
    FeatureBasePtr      f = C->addFeature(std::make_shared<FeatureBase>("ODOM 2D", Vector3s::Zero(), Matrix3s::Identity()));
    ConstraintPose2DPtr c = std::static_pointer_cast<ConstraintPose2D>(f->addConstraint(std::make_shared<ConstraintPose2D>(f)));

    // update solver
    solver_manager_ptr->update();

    // remove constraint
    P->removeConstraint(c);

    // update solver
    solver_manager_ptr->update();

    // remove constraint
    P->removeConstraint(c);

    // update solver
    solver_manager_ptr->update();

    // check constraint
    ASSERT_FALSE(solver_manager_ptr->isConstraintRegistered(c));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

