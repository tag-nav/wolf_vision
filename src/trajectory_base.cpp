#include "trajectory_base.h"

TrajectoryBase::TrajectoryBase() : 
    NodeLinked(MID, "TRAJECTORY")
{
    //
}

TrajectoryBase::~TrajectoryBase()
{
	//std::cout << "deleting TrajectoryBase " << nodeId() << std::endl;
}

void TrajectoryBase::addFrame(FrameBase* _frame_ptr)
{
	addDownNode(_frame_ptr);
}

void TrajectoryBase::removeFrame(const FrameBaseIter& _frame_iter)
{
	removeDownNode(_frame_iter);
}

FrameBaseList* TrajectoryBase::getFrameListPtr()
{
    return getDownNodeListPtr();
}

FrameBase* TrajectoryBase::getLastFramePtr()
{
    return getDownNodeListPtr()->back();
}

void TrajectoryBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(auto fr_it = getFrameListPtr()->begin(); fr_it != getFrameListPtr()->end(); ++fr_it)
		(*fr_it)->getConstraintList(_ctr_list);
}
