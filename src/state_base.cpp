
#include "state_base.h"

StateBase::StateBase(Eigen::VectorXs& _st_remote, const unsigned int _idx) :
			state_ptr_(_st_remote.data() + _idx),
			status_(ST_ESTIMATED)
{
	//
}


StateBase::StateBase(WolfScalar* _st_ptr) :
			state_ptr_(_st_ptr),
			status_(ST_ESTIMATED)
{
	//
}
                
StateBase::~StateBase()
{
    //
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
}

void StateBase::printTabs(unsigned int _ntabs, std::ostream& _ost) const
{
	for (unsigned int i = 0; i < _ntabs; i++) _ost << "\t";
}
