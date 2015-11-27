
#include "state_block.h"

StateBlock::StateBlock(const Eigen::VectorXs _state, StateType _type, bool _fixed) :
			type_(_type),
			state_(_state),
			fixed_(_fixed)
{
	//
}

StateBlock::~StateBlock()
{
	//std::cout << "deleting StateBase " << nodeId() << std::endl;
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

StateType StateBlock::getType() const
{
	return type_;
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
