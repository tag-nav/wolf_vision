#include "base/solver/solver_manager.h"
#include "base/trajectory_base.h"
#include "base/map_base.h"
#include "base/landmark/landmark_base.h"

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
    //std::cout << wolf_problem_->getFactorNotificationList().size() << " factor notifications" << std::endl;

	// REMOVE CONSTRAINTS
	auto ctr_notification_it = wolf_problem_->getFactorNotificationList().begin();
	while ( ctr_notification_it != wolf_problem_->getFactorNotificationList().end() )
		if (ctr_notification_it->notification_ == REMOVE)
		{
			removeFactor(ctr_notification_it->factor_ptr_);
			ctr_notification_it = wolf_problem_->getFactorNotificationList().erase(ctr_notification_it);
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
    while (!wolf_problem_->getFactorNotificationList().empty())
    {
        switch (wolf_problem_->getFactorNotificationList().front().notification_)
        {
            case ADD:
            {
                addFactor(wolf_problem_->getFactorNotificationList().front().factor_ptr_);
                break;
            }
            default:
                throw std::runtime_error("SolverManager::update: Factor notification must be ADD or REMOVE.");
        }
        wolf_problem_->getFactorNotificationList().pop_front();
    }

    assert(wolf_problem_->getFactorNotificationList().empty() && "wolf problem's factors notification list not empty after update");
    assert(wolf_problem_->getStateBlockNotificationList().empty() && "wolf problem's state_blocks notification list not empty after update");
}

ProblemPtr SolverManager::getProblem()
{
    return wolf_problem_;
}

} // namespace wolf

