/*
 * gtest_solver_manager.cpp
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

#include <iostream>

using namespace wolf;
using namespace Eigen;

WOLF_PTR_TYPEDEFS(SolverManagerWrapper);
class SolverManagerWrapper : public SolverManager
{
    public:
        std::list<ConstraintBasePtr> constraints_;
        std::map<StateBlockPtr,bool> state_block_fixed_;

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

        virtual void computeCovariances(const CovarianceBlocksToBeComputed blocks){};
        virtual void computeCovariances(const StateBlockList& st_list){};

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
        };
        virtual void removeStateBlock(const StateBlockPtr& state_ptr)
        {
            state_block_fixed_.erase(state_ptr);
        };
        virtual void updateStateBlockStatus(const StateBlockPtr& state_ptr)
        {
            state_block_fixed_[state_ptr] = state_ptr->isFixed();
        };
};

TEST(SolverManager, Create)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // check double ointers to branches
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

    // update solver
    solver_manager_ptr->update();

    // check stateblock unfixed
    ASSERT_FALSE(solver_manager_ptr->isStateBlockFixed(sb_ptr));

    // Fix frame
    sb_ptr->fix();

    // update stateblock
    P->updateFixStateBlockPtr(sb_ptr);

    // update solver manager
    solver_manager_ptr->update();

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

    // Fix state block
    sb_ptr->fix();

    // update stateblock
    P->updateFixStateBlockPtr(sb_ptr);

    // update solver manager
    solver_manager_ptr->update();

    // check stateblock fixed
    ASSERT_TRUE(solver_manager_ptr->isStateBlockRegistered(sb_ptr));
    ASSERT_TRUE(solver_manager_ptr->isStateBlockFixed(sb_ptr));
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
    P->removeStateBlockPtr(sb_ptr);

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
    P->removeStateBlockPtr(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    // check stateblock
    ASSERT_FALSE(solver_manager_ptr->isStateBlockRegistered(sb_ptr));
}

TEST(SolverManager, RemoveUpdateStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Create State block
    Vector2s state; state << 1, 2;
    StateBlockPtr sb_ptr = std::make_shared<StateBlock>(state);

    // add stateblock
    P->addStateBlock(sb_ptr);

    // remove state_block
    P->removeStateBlockPtr(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    // Fix state block
    sb_ptr->fix();

    ASSERT_DEATH({
        // update stateblock
        P->updateFixStateBlockPtr(sb_ptr);

        // update solver manager
        solver_manager_ptr->update();
    },"");
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
    P->removeStateBlockPtr(sb_ptr);

    // update solver
    solver_manager_ptr->update();

    // remove state_block
    P->removeStateBlockPtr(sb_ptr);

    // update solver manager
    solver_manager_ptr->update();
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
    P->removeConstraintPtr(c);

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

    ASSERT_TRUE(P->getConstraintNotificationList().front().constraint_ptr_ == c);

    // add constraint
    P->removeConstraintPtr(c);

    ASSERT_TRUE(P->getConstraintNotificationList().empty());

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
    P->removeConstraintPtr(c);

    // update solver
    solver_manager_ptr->update();

    // remove constraint
    P->removeConstraintPtr(c);

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

