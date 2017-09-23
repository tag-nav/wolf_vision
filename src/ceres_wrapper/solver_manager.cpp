#include "solver_manager.h"
#include "../trajectory_base.h"
#include "../map_base.h"
#include "../landmark_base.h"

namespace wolf {

SolverManager::SolverManager(ProblemPtr _wolf_problem) :
    wolf_problem_(_wolf_problem)
{
}

SolverManager::~SolverManager()
{
}

void SolverManager::update()
{
    //std::cout << "SolverManager: updating... " << std::endl;
    //std::cout << wolf_problem_->getStateBlockNotificationList().size() << " state block notifications" << std::endl;
    //std::cout << wolf_problem_->getConstraintNotificationList().size() << " constraint notifications" << std::endl;

	// REMOVE CONSTRAINTS
	auto ctr_notification_it = wolf_problem_->getConstraintNotificationList().begin();
	while ( ctr_notification_it != wolf_problem_->getConstraintNotificationList().end() )
		if (ctr_notification_it->notification_ == REMOVE)
		{
			removeConstraint(ctr_notification_it->constraint_ptr_);
			ctr_notification_it = wolf_problem_->getConstraintNotificationList().erase(ctr_notification_it);
		}
		else
			ctr_notification_it++;

	// REMOVE STATE BLOCKS
	auto state_notification_it = wolf_problem_->getStateBlockNotificationList().begin();
	while ( state_notification_it != wolf_problem_->getStateBlockNotificationList().end() )
		if (state_notification_it->notification_ == REMOVE)
		{
			removeStateBlock(state_notification_it->state_block_ptr_);
			state_notification_it = wolf_problem_->getStateBlockNotificationList().erase(state_notification_it);
		}
		else
			state_notification_it++;

    // ADD/UPDATE STATE BLOCKS
    while (!wolf_problem_->getStateBlockNotificationList().empty())
    {
        switch (wolf_problem_->getStateBlockNotificationList().front().notification_)
        {
            case ADD:
            {
                addStateBlock(wolf_problem_->getStateBlockNotificationList().front().state_block_ptr_);
                if (wolf_problem_->getStateBlockNotificationList().front().state_block_ptr_->isFixed())
                    updateStateBlockStatus(wolf_problem_->getStateBlockNotificationList().front().state_block_ptr_);
                break;
            }
            case UPDATE:
            {
                updateStateBlockStatus(wolf_problem_->getStateBlockNotificationList().front().state_block_ptr_);
                break;
            }
            default:
                throw std::runtime_error("SolverManager::update: State Block notification must be ADD, UPATE or REMOVE.");
        }
        wolf_problem_->getStateBlockNotificationList().pop_front();
    }
    // ADD CONSTRAINTS
    while (!wolf_problem_->getConstraintNotificationList().empty())
    {
        switch (wolf_problem_->getConstraintNotificationList().front().notification_)
        {
            case ADD:
            {
                addConstraint(wolf_problem_->getConstraintNotificationList().front().constraint_ptr_);
                break;
            }
            default:
                throw std::runtime_error("SolverManager::update: Constraint notification must be ADD or REMOVE.");
        }
        wolf_problem_->getConstraintNotificationList().pop_front();
    }

    assert(wolf_problem_->getConstraintNotificationList().empty() && "wolf problem's constraints notification list not empty after update");
    assert(wolf_problem_->getStateBlockNotificationList().empty() && "wolf problem's state_blocks notification list not empty after update");
}

wolf::ProblemPtr SolverManager::getProblemPtr()
{
    return wolf_problem_;
}

} // namespace wolf

