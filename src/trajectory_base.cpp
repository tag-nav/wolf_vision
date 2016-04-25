#include "trajectory_base.h"
#include "frame_base.h"
#include "problem.h"

namespace wolf {

TrajectoryBase::TrajectoryBase(FrameStructure _frame_structure) :
    NodeLinked(MID, "TRAJECTORY"),
    frame_structure_(_frame_structure), fixed_size_(0)
{
    //
}

TrajectoryBase::~TrajectoryBase()
{
    //std::cout << "deleting TrajectoryBase " << nodeId() << std::endl;
}

FrameBase* TrajectoryBase::addFrame(FrameBase* _frame_ptr)
{
	addDownNode(_frame_ptr);
    if (_frame_ptr->isKey())
        _frame_ptr->registerNewStateBlocks();

    return _frame_ptr;
}

void TrajectoryBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(auto fr_it = getFrameListPtr()->begin(); fr_it != getFrameListPtr()->end(); ++fr_it)
		(*fr_it)->getConstraintList(_ctr_list);
}

} // namespace wolf
