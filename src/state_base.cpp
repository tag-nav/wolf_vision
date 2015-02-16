
#include "state_base.h"

StateBase::StateBase(Eigen::VectorXs& _st_remote, const unsigned int _idx) :
			state_ptr_(_st_remote.data() + _idx)
{
	//
}


StateBase::StateBase(WolfScalar* _st_ptr) :
			state_ptr_(_st_ptr)
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

void StateBase::printTabs(unsigned int _ntabs, std::ostream& _ost) const
{
	for (unsigned int i = 0; i < _ntabs; i++) _ost << "\t";
}
