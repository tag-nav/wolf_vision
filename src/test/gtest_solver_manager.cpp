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
#include "../solver/solver_manager.h"

#include <iostream>

using namespace wolf;
using namespace Eigen;

WOLF_PTR_TYPEDEFS(SolverManagerWrapper);
class SolverManagerWrapper : public SolverManager
{
    public:
        SolverManagerWrapper(const ProblemPtr& wolf_problem) :
            SolverManagerWrapper(wolf_problem)
        {

        };

        bool isStateBlockRegistered(const StateBlockPtr& st) const
        {
            return state_blocks_.find(st)!=state_blocks_.end();
        };

        virtual void computeCovariances(const CovarianceBlocksToBeComputed blocks){};
        virtual void computeCovariances(const StateBlockList& st_list){};

    protected:

        virtual std::string solveImpl(const ReportVerbosity report_level){ return std::string("");};
        virtual void addConstraint(const ConstraintBasePtr& ctr_ptr){};
        virtual void removeConstraint(const ConstraintBasePtr& ctr_ptr){};
        virtual void addStateBlock(const StateBlockPtr& state_ptr){};
        virtual void removeStateBlock(const StateBlockPtr& state_ptr){};
        virtual void updateStateBlockStatus(const StateBlockPtr& state_ptr){};
};

TEST(SolverManager, Create)
{
    ProblemPtr P = Problem::create("PO 2D");
    //CeresManagerPtr ceres_manager_ptr = std::make_shared<CeresManager>(P);
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // check double ointers to branches
    ASSERT_EQ(P, solver_manager_ptr->getProblemPtr());
}

TEST(SolverManager, AddStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    //CeresManagerPtr ceres_manager_ptr = std::make_shared<CeresManager>(P);
    SolverManagerWrapperPtr solver_manager_ptr = std::make_shared<SolverManagerWrapper>(P);

    // Set prior -> frame + constraint
    Vector3s F1_state; F1_state << 1, 2, 3;
    FrameBasePtr F1 = P->emplaceFrame(KEY_FRAME, F1_state,TimeStamp());

    // update solver manager
    solver_manager_ptr->update();

    // check stateblock
    ASSERT_TRUE(solver_manager_ptr->isStateBlockRegistered(F1->getStateBlockPtr(0)));
    ASSERT_TRUE(solver_manager_ptr->isStateBlockRegistered(F1->getStateBlockPtr(1)));
    ASSERT_EQ(solver_manager_ptr->getAssociatedMemBlock(F1->getStateBlockPtr(0)), F1->getStateBlockPtr(0)->getState());
}

/*TEST(SolverManager, AddPrior)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerPtr ceres_manager_ptr = std::make_shared<CeresManager>(P);

    // Set prior -> frame + constraint
    Vector3s F1_state; F1_state << 1, 2, 3;
    FrameBasePtr F1 = P->setPrior(Vector3s::Zero(),Matrix3s::Identity(),TimeStamp());
    ConstraintBasePtr C1 = F1->getCaptureList().front()->getFeatureList().front()->getConstraintList().front();

    // update solver manager
    ceres_manager_ptr->update();

    // check stateblock
    ASSERT_EQ(ceres_manager_ptr->getAssociatedMemBlock(F1->getStateBlockPtr(0)), F1->getStateBlockPtr(0)->getState());

    // solve
    ceres_manager_ptr->solve();
}

TEST(SolverManager, UpdateStateBlock)
{
    ProblemPtr P = Problem::create("PO 2D");
    CeresManagerPtr ceres_manager_ptr = std::make_shared<CeresManager>(P);

    // Set prior -> frame + constraint
    Vector3s F1_state; F1_state << 1, 2, 3;
    FrameBasePtr F1 = P->setPrior(F1_state,Matrix3s::Identity(),TimeStamp());
    ConstraintBasePtr C1 = F1->getCaptureList().front()->getFeatureList().front()->getConstraintList().front();

    // update solver manager
    ceres_manager_ptr->update();

    // check stateblock
    ASSERT_EQ(ceres_manager_ptr->getAssociatedMemBlock(F1->getStateBlockPtr(0)), F1->getStateBlockPtr(0)->getState());

    // solve
    ceres_manager_ptr->solve();

    // Change state and fix
    F1->setState(Vector3s::Zero());
    F1->fix();

    // update solver manager
    ceres_manager_ptr->update();

}*/

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

