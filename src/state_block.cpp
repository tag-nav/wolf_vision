#include "base/state_block.h"
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

    // Flag
    if (_notify)
        state_updated_.store(true);
}

void StateBlock::setFixed(bool _fixed)
{
    // Flag
    if (fixed_.load() != _fixed)
        fix_updated_.store(true);

    // set
    fixed_.store(_fixed);
}

//void StateBlock::addToProblem(ProblemPtr _problem_ptr)
//{
//    _problem_ptr->addStateBlock(shared_from_this());
//}
//
//void StateBlock::removeFromProblem(ProblemPtr _problem_ptr)
//{
//    _problem_ptr->removeStateBlockPtr(shared_from_this());
//}

}
