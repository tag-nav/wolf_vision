#include "base/solver/solver_manager.h"
#include "base/trajectory/trajectory_base.h"
#include "base/map/map_base.h"
#include "base/landmark/landmark_base.h"

namespace wolf {

SolverManager::SolverManager(const ProblemPtr& _wolf_problem) :
          wolf_problem_(_wolf_problem)
{
    assert(_wolf_problem != nullptr && "Passed a nullptr ProblemPtr.");
}

void SolverManager::update()
{
    // Consume notification maps
    auto fac_notification_map = wolf_problem_->consumeFactorNotificationMap();
    auto sb_notification_map = wolf_problem_->consumeStateBlockNotificationMap();

    // REMOVE CONSTRAINTS
    for (auto fac_notification_it = fac_notification_map.begin();
         fac_notification_it != fac_notification_map.end();
         /* nothing, next is handled within the for */)
    {
        if (fac_notification_it->second == REMOVE)
        {
            removeFactor(fac_notification_it->first);
            fac_notification_it = fac_notification_map.erase(fac_notification_it);
        }
        else
            fac_notification_it++;
    }

    // ADD/REMOVE STATE BLOCS
    while ( !sb_notification_map.empty() )
    {
        StateBlockPtr state_ptr = sb_notification_map.begin()->first;

        if (sb_notification_map.begin()->second == ADD)
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
                WOLF_DEBUG("Tried to add an already added !");
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
        sb_notification_map.erase(sb_notification_map.begin());
    }

    // ADD CONSTRAINTS
    while (!fac_notification_map.empty())
    {
        assert(fac_notification_map.begin()->second == ADD && "unexpected factor notification value after all REMOVE have been processed, this should be ADD");

        // add factor
        addFactor(fac_notification_map.begin()->first);
        // remove notification
        fac_notification_map.erase(fac_notification_map.begin());
    }

    // UPDATE STATE BLOCKS (state, fix or local parameterization)
    for (auto state_pair : state_blocks_)
    {
        auto state_ptr = state_pair.first;

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
}

wolf::ProblemPtr SolverManager::getProblem()
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
    /// JV: I do not see a problem here, the solver provides the optimal state given the factors, if someone changed the state during optimization, it will be overwritten by the optimal one.

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
