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

// const inline FrameBaseList* TrajectoryBase::frameList() const
// {
//     return downNodeListPtr();
// }

void TrajectoryBase::printSelf(unsigned int _ntabs, std::ostream& _ost) const
{
	NodeLinked::printSelf(_ntabs, _ost);
}
