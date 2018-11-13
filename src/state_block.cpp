#include "state_block.h"
namespace wolf
{

void StateBlock::setState(const Eigen::VectorXs& _state)
{
    assert(_state.size() == state_.size());
    {
        std::lock_guard<std::mutex> lock(mut_state_);
        state_ = _state;
        state_size_ = state_.size();
    }
    if (getProblem() != nullptr)
        getProblem()->updateStateStateBlockPtr(shared_from_this());
}

StateBlock::Notifications StateBlock::consumeNotifications() const
{
    std::lock_guard<std::mutex> lock(notifictions_mut_);
    return std::move(notifications_);
}

void StateBlock::printNotifications() const
{
    WOLF_TRACE("SB Notifications for: ", shared_from_this())
    for (auto notif : notifications_)
    {
        switch (notif)
        {
            case Notification::ADD:
                WOLF_TRACE("   ADD")
                break;
            case Notification::REMOVE:
                WOLF_TRACE("   REMOVE")
                break;
            case Notification::UPDATE_FIX:
                WOLF_TRACE("   UPDATE_FIX")
                break;
            case Notification::UPDATE_STATE:
                WOLF_TRACE("   UPDATE_STATE")
                break;
        }
    }

}

}
