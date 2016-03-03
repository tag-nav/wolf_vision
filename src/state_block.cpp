
#include "state_block.h"

StateBlock::StateBlock(const unsigned int _size, bool _fixed) :
        state_(_size),
        fixed_(_fixed),
        local_param_ptr_(nullptr)
{
    //
}

StateBlock::StateBlock(const Eigen::VectorXs _state, bool _fixed) :
        state_(_state),
        fixed_(_fixed),
        local_param_ptr_(nullptr)
{
	//
}

StateBlock::~StateBlock()
{
    //
}

WolfScalar* StateBlock::getPtr()
{
	return state_.data();
}

Eigen::VectorXs StateBlock::getVector()
{
	return state_;
}

void StateBlock::setVector(const Eigen::VectorXs& _state)
{
    assert(_state.size() == state_.size());
    state_ = _state;
}

unsigned int StateBlock::getSize() const
{
    return state_.size();
}

bool StateBlock::isFixed() const
{
    return fixed_;
}

void StateBlock::fix()
{
    fixed_ = true;
}

void StateBlock::unfix()
{
    fixed_ = false;
}

void StateBlock::print(unsigned int _ntabs, std::ostream& _ost) const
{
}

bool StateBlock::hasLocalParametrization()
{
    return (local_param_ptr_ != nullptr);
}

LocalParametrizationBase* StateBlock::getLocalParametrizationPtr()
{
    return local_param_ptr_;
}

