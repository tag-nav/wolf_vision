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
            getProblem()->notifyStateBlock(shared_from_this());
    }
}

void StateBlock::setFixed(bool _fixed)
{
    fixed_.store(_fixed);
    // Notify
    addNotification(StateBlock::Notification::UPDATE_FIX);
    if (getProblem() != nullptr)
        getProblem()->notifyStateBlock(shared_from_this());
}

void StateBlock::addNotification(const StateBlock::Notification _new_notification)
{
    std::lock_guard<std::mutex> lock(notifictions_mut_);
    if (_new_notification == Notification::ADD)
    {
        // When an ADD arrives, the state is already the newest,
        // thus old instructions can be cleared
        if (shared_from_this()->notifications_.size() > 0)
            notifications_.clear();

        // Push ADD notification to the front
        notifications_.emplace_front(Notification::ADD);
    }
    else if (_new_notification == Notification::REMOVE)
    {
        // If we want to remove a block that still has an ADD instruction
        // we can just clear all notifications and just keep the remove
        if (!notifications_.empty() && notifications_.front() == Notification::ADD)
            notifications_.clear();
        else
        {
            notifications_.clear();
            notifications_.emplace_back(Notification::REMOVE);
        }
    }
    else
        // UPDATE_FIX; UPDATE_STATE
        notifications_.emplace_back(_new_notification);
}

StateBlock::Notifications StateBlock::consumeNotifications() const
{
    std::lock_guard<std::mutex> lock(notifictions_mut_);
    return std::move(notifications_);
}

StateBlock::Notifications StateBlock::getNotifications() const
{
    std::lock_guard<std::mutex> lock(notifictions_mut_);
    return notifications_;
}

void StateBlock::printNotifications() const
{
    WOLF_TRACE("SB Notifications for: ", shared_from_this());
    std::lock_guard<std::mutex> lock(notifictions_mut_);
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
