#include "trajectory_base.h"

namespace wolf {

TrajectoryBase::TrajectoryBase(FrameStructure _frame_structure) :
    NodeLinked(MID, "TRAJECTORY"),
    frame_structure_(_frame_structure), last_key_frame_ptr_(nullptr), fixed_size_(0)
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
    {
        _frame_ptr->registerNewStateBlocks();
        if (last_key_frame_ptr_ == nullptr || last_key_frame_ptr_->getTimeStamp() < _frame_ptr->getTimeStamp())
            last_key_frame_ptr_ = _frame_ptr;
    }

    return _frame_ptr;
}

void TrajectoryBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(auto fr_it = getFrameListPtr()->begin(); fr_it != getFrameListPtr()->end(); ++fr_it)
		(*fr_it)->getConstraintList(_ctr_list);
}

} // namespace wolf
