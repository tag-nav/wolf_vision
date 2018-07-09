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
  auto ctr_notification_it = wolf_problem_->getConstraintNotificationList().begin();
  while ( ctr_notification_it != wolf_problem_->getConstraintNotificationList().end() )
  {
    if (ctr_notification_it->notification_ == REMOVE)
    {
      removeConstraint(ctr_notification_it->constraint_ptr_);
      ctr_notification_it = wolf_problem_->getConstraintNotificationList().erase(ctr_notification_it);
    }
    else
      ctr_notification_it++;
  }

  StateBlockList& states = wolf_problem_->getNotifiedStateBlockList();

  for (StateBlockPtr& state : states)
  {
    const auto notifications = state->consumeNotifications();

    for (const auto notif : notifications)
    {
      switch (notif)
      {
      case StateBlock::Notification::ADD:
      {
        const bool registered = state_blocks_.find(state)!=state_blocks_.end();

//        const auto p = state_blocks_.emplace(state, state->getState());

        // call addStateBlock only if first time added.
        if (!registered)
        {
          state_blocks_.emplace(state, state->getState());
          addStateBlock(state);
        }

        WOLF_DEBUG_COND(registered, "Tried adding an already registered StateBlock.");

        break;
      }
      case StateBlock::Notification::STATE_UPDATE:
      {
        WOLF_DEBUG_COND(state_blocks_.find(state)==state_blocks_.end(),
                        "Updating the state of an unregistered StateBlock !");

        assert(state_blocks_.find(state)!=state_blocks_.end() &&
            "Updating the state of an unregistered StateBlock !");

        Eigen::VectorXs new_state = state->getState();
        std::copy(new_state.data(),new_state.data()+new_state.size(),getAssociatedMemBlockPtr(state));

        break;
      }
      case StateBlock::Notification::FIX_UPDATE:
      {
        WOLF_DEBUG_COND(state_blocks_.find(state)==state_blocks_.end(),
                        "Updating the fix state of an unregistered StateBlock !");

        assert(state_blocks_.find(state)!=state_blocks_.end() &&
            "Updating the fix state of an unregistered StateBlock !");

        if (state_blocks_.find(state)!=state_blocks_.end())
        {
            updateStateBlockStatus(state);
        }

        break;
      }
      case StateBlock::Notification::REMOVE:
      {
        WOLF_DEBUG_COND(state_blocks_.find(state)==state_blocks_.end(),
                        "Tried to remove a StateBlock that was not added !");

        if (state_blocks_.find(state)!=state_blocks_.end())
        {
            removeStateBlock(state);
            state_blocks_.erase(state);
        }

        break;
      }
      default:
        throw std::runtime_error("SolverManager::update: State Block notification "
                                 "must be ADD, STATE_UPDATE, FIX_UPDATE or REMOVE.");
      }
    }
  }

  states.clear();

  // ADD CONSTRAINTS
  while (!wolf_problem_->getConstraintNotificationList().empty())
  {
    switch (wolf_problem_->getConstraintNotificationList().front().notification_)
    {
    case Notification::ADD:
    {
      addConstraint(wolf_problem_->getConstraintNotificationList().front().constraint_ptr_);

      break;
    }
    default:
      throw std::runtime_error("SolverManager::update:"
                               " Constraint notification must be ADD or REMOVE.");
    }

    wolf_problem_->getConstraintNotificationList().pop_front();
  }

  assert(wolf_problem_->getConstraintNotificationList().empty() &&
         "wolf problem's constraints notification list not empty after update");
  assert(wolf_problem_->getNotifiedStateBlockList().empty() &&
         "wolf problem's state_blocks notification list not empty after update");
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

  std::map<StateBlockPtr, Eigen::VectorXs>::iterator it = state_blocks_.begin(),
                                                     it_end = state_blocks_.end();
  for (; it != it_end; ++it)
  {
    // Avoid usuless copies
    if (!it->first->isFixed())
      it->first->setState(it->second);
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
