
#include "state_base.h"

StateBase::StateBase(WolfScalar* _st_ptr, unsigned int _size, StateType _type, StateStatus _status) :
			type_(_type),
			state_ptr_(_st_ptr),
			size_(_size),
			status_(_status),
			pending_status_(ADD_PENDING)
{
	//
}

StateBase::~StateBase()
{
	//std::cout << "deleting StateBase " << nodeId() << std::endl;
}

WolfScalar* StateBase::getPtr()
{
	return state_ptr_;
}

void StateBase::setPtr(WolfScalar* _new_ptr)
{
	state_ptr_ = _new_ptr;
}

StateStatus StateBase::getStateStatus() const
{
	return status_;
}

void StateBase::setStateStatus(const StateStatus& _status)
{
	status_=_status;
	if (getPendingStatus() != ADD_PENDING)
		setPendingStatus(UPDATE_PENDING);
}

PendingStatus StateBase::getPendingStatus() const
{
	return pending_status_;
}

unsigned int StateBase::getStateSize() const {
	return size_;
}

void StateBase::setPendingStatus(PendingStatus _pending)
{
	pending_status_ = _pending;
}

Eigen::Map<const Eigen::VectorXs> StateBase::getVector() const {
	return Eigen::Map<const Eigen::VectorXs> (state_ptr_,size_);
}

StateType StateBase::getStateType() const {
	return type_;
}

void StateBase::print(unsigned int _ntabs, std::ostream& _ost) const {
}
