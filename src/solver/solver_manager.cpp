#include "solver_manager.h"
#include "../trajectory_base.h"
#include "../map_base.h"
#include "../landmark_base.h"

namespace wolf {

SolverManager::SolverManager(const ProblemPtr& _wolf_problem) :
          wolf_problem_(_wolf_problem)
{
    assert(_wolf_problem != nullptr && "Passed a nullptr ProblemPtr.");
}

void SolverManager::update()
{
    // REMOVE CONSTRAINTS
    auto ctr_notification_it = wolf_problem_->getConstraintNotificationMap().begin();
    while ( ctr_notification_it != wolf_problem_->getConstraintNotificationMap().end() )
    {
        if (ctr_notification_it->second == REMOVE)
        {
            removeConstraint(ctr_notification_it->first);
            ctr_notification_it = wolf_problem_->getConstraintNotificationMap().erase(ctr_notification_it);
        }
        else
            ctr_notification_it++;
    }

    // ADD/REMOVE STATE BLOCS
    auto sb_notification_it = wolf_problem_->getStateBlockNotificationMap().begin();
    while ( sb_notification_it != wolf_problem_->getStateBlockNotificationMap().end() )
    {
        StateBlockPtr state_ptr = sb_notification_it->first;

        if (sb_notification_it->second == ADD)
        {
            // only add if not added
            if (state_blocks_.find(state_ptr) == state_blocks_.end())
            {
                state_blocks_.emplace(state_ptr, state_ptr->getState());
                addStateBlock(state_ptr);
                // A state_block is added with its last state_ptr, status and local_param, thus, no update is needed for any of those things -> reset flags
                state_ptr->resetStateUpdated();
                state_ptr->resetFixUpdated();
                state_ptr->resetLocalParamUpdated();
            }
            else
            {
                WOLF_DEBUG("Tried adding an already registered StateBlock.");
            }
        }
        else
        {
            // only remove if it exists
            if (state_blocks_.find(state_ptr)!=state_blocks_.end())
            {
                removeStateBlock(state_ptr);
                state_blocks_.erase(state_ptr);
            }
            else
            {
                WOLF_DEBUG("Tried to remove a StateBlock that was not added !");
            }
        }
        // next notification
        sb_notification_it = wolf_problem_->getStateBlockNotificationMap().erase(sb_notification_it);
    }

    // ADD CONSTRAINTS
    ctr_notification_it = wolf_problem_->getConstraintNotificationMap().begin();
    while (ctr_notification_it != wolf_problem_->getConstraintNotificationMap().end())
    {
        assert(wolf_problem_->getConstraintNotificationMap().begin()->second == ADD && "unexpected constraint notification value after all REMOVE have been processed, this should be ADD");

        addConstraint(wolf_problem_->getConstraintNotificationMap().begin()->first);
        ctr_notification_it = wolf_problem_->getConstraintNotificationMap().erase(ctr_notification_it);
    }

    // UPDATE STATE BLOCKS (state, fix or local parameterization)
    for (auto state_ptr : wolf_problem_->getStateBlockList())
    {
        assert(state_blocks_.find(state_ptr)!=state_blocks_.end() && "Updating the state of an unregistered StateBlock !");

        // state update
        if (state_ptr->stateUpdated())
        {
            Eigen::VectorXs new_state = state_ptr->getState();
            // We assume the same size for the states in both WOLF and the solver.
            std::copy(new_state.data(),new_state.data()+new_state.size(),getAssociatedMemBlockPtr(state_ptr));
            // reset flag
            state_ptr->resetStateUpdated();
        }
        // fix update
        if (state_ptr->fixUpdated())
        {
            updateStateBlockStatus(state_ptr);
            // reset flag
            state_ptr->resetFixUpdated();
        }
        // local parameterization update
        if (state_ptr->localParamUpdated())
        {
            updateStateBlockLocalParametrization(state_ptr);
            // reset flag
            state_ptr->resetLocalParamUpdated();
        }
    }

    assert(wolf_problem_->getConstraintNotificationMap().empty() && "wolf problem's constraints notification map not empty after update");
    assert(wolf_problem_->getStateBlockNotificationMap().empty() && "wolf problem's state_blocks notification map not empty after update");
}

wolf::ProblemPtr SolverManager::getProblemPtr()
{
    return wolf_problem_;
}

std::string SolverManager::solve(const ReportVerbosity report_level)
{
    // update problem
    update();

    std::string report = solveImpl(report_level);

    // update StateBlocks with optimized state value.
    /// @todo whatif someone has changed the state notification during opti ??
    /// JV: I do not see a problem here, the solver provides the optimal state given the constraints, if someone changed the state during optimization, it will be overwritten by the optimal one.

    std::map<StateBlockPtr, Eigen::VectorXs>::iterator it = state_blocks_.begin(),
            it_end = state_blocks_.end();
    for (; it != it_end; ++it)
    {
        // Avoid usuless copies
        if (!it->first->isFixed())
            it->first->setState(it->second, false); // false = do not raise the flag state_updated_
    }

    return report;
}

Eigen::VectorXs& SolverManager::getAssociatedMemBlock(const StateBlockPtr& state_ptr)
{
    auto it = state_blocks_.find(state_ptr);

    if (it == state_blocks_.end())
        throw std::runtime_error("Tried to retrieve the memory block of an unregistered StateBlock !");

    return it->second;
}

Scalar* SolverManager::getAssociatedMemBlockPtr(const StateBlockPtr& state_ptr)
{
    auto it = state_blocks_.find(state_ptr);

    if (it == state_blocks_.end())
        throw std::runtime_error("Tried to retrieve the memory block of an unregistered StateBlock !");

    return it->second.data();
}

} // namespace wolf
