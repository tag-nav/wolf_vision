#include "state_block.h"
namespace wolf
{

void StateBlock::setState(const Eigen::VectorXs& _state, const bool _notify)
{
    assert(_state.size() == state_.size());
    {
        std::lock_guard<std::mutex> lock(mut_state_);
        state_ = _state;
        state_size_ = state_.size();
    }

    // Notify
    if (_notify)
    {
        addNotification(StateBlock::Notification::UPDATE_STATE);
        if (getProblem() != nullptr)
            getProblem()->notifyStateBlock(shared_from_this(), StateBlock::Notification::UPDATE_STATE);
    }
}

void StateBlock::setFixed(bool _fixed)
{
    fixed_.store(_fixed);
    // Notify
    addNotification(StateBlock::Notification::UPDATE_FIX);
    if (getProblem() != nullptr)
        getProblem()->notifyStateBlock(shared_from_this(), StateBlock::Notification::UPDATE_FIX);
}

StateBlock::Notifications StateBlock::consumeNotifications() const
{
    std::lock_guard<std::mutex> lock(notifictions_mut_);
    return std::move(notifications_);
}

StateBlock::Notifications StateBlock::getNotifications() const
{
    return notifications_;
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
