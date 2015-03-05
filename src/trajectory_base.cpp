#include "trajectory_base.h"

TrajectoryBase::TrajectoryBase() : 
    NodeLinked(MID, "TRAJECTORY")
{
    //
}

TrajectoryBase::~TrajectoryBase()
{
    //
}

void TrajectoryBase::addFrame(FrameBaseShPtr _frame_ptr)
{
	addDownNode(_frame_ptr);
}

FrameBaseList* TrajectoryBase::getFrameListPtr()
{
    return getDownNodeListPtr();
}

void TrajectoryBase::getConstraintList(ConstraintBasePtrList & _ctr_list)
{
	for(auto fr_it = getFrameListPtr()->begin(); fr_it != getFrameListPtr()->end(); ++fr_it)
		(*fr_it)->getConstraintList(_ctr_list);
}

// const inline FrameBaseList* TrajectoryBase::frameList() const
// {
//     return downNodeListPtr();
// }

void TrajectoryBase::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
	NodeLinked::printSelf(_ntabs, _ost);
}
