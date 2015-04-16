
#include "state_base.h"

StateBase::StateBase(WolfScalar* _st_ptr) :
			NodeBase("STATE"),
			state_ptr_(_st_ptr),
			status_(ST_ESTIMATED)
{
	//
}

StateBase::StateBase(Eigen::VectorXs& _st_remote, const unsigned int _idx) :
            NodeBase("STATE"),
            state_ptr_(_st_remote.data() + _idx),
            status_(ST_ESTIMATED)
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
