#include "trajectory_base.h"

namespace wolf {

TrajectoryBase::TrajectoryBase(FrameStructure _frame_structure) :
    NodeLinked(MID, "TRAJECTORY"),
    frame_structure_(_frame_structure), last_key_frame_ptr_(nullptr)
{
    //
}

TrajectoryBase::~TrajectoryBase()
{
    //std::cout << "deleting TrajectoryBase " << nodeId() << std::endl;
}

FrameBase* TrajectoryBase::addFrame(FrameBase* _frame_ptr)
{
    if (_frame_ptr->isKey())
    {
        _frame_ptr->registerNewStateBlocks();
        if (last_key_frame_ptr_ == nullptr || last_key_frame_ptr_->getTimeStamp() < _frame_ptr->getTimeStamp())
            last_key_frame_ptr_ = _frame_ptr;

        insertDownNode(_frame_ptr, computeFrameOrder(_frame_ptr));
    }
    else
        addDownNode(_frame_ptr);

    return _frame_ptr;
}

void TrajectoryBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(auto fr_it = getFrameListPtr()->begin(); fr_it != getFrameListPtr()->end(); ++fr_it)
		(*fr_it)->getConstraintList(_ctr_list);
}

void TrajectoryBase::sortFrame(FrameBase* _frame_ptr)
{
    moveDownNode(_frame_ptr, computeFrameOrder(_frame_ptr));
}

FrameBaseIter TrajectoryBase::computeFrameOrder(FrameBase* _frame_ptr)
{
    for (auto frm_rit = getFrameListPtr()->rbegin(); frm_rit != getFrameListPtr()->rend(); frm_rit++)
        if ((*frm_rit)!= _frame_ptr && (*frm_rit)->isKey() && (*frm_rit)->getTimeStamp() < _frame_ptr->getTimeStamp())
            return frm_rit.base();
    return getFrameListPtr()->begin();
}

} // namespace wolf
