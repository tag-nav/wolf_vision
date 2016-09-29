#include "trajectory_base.h"
#include "frame_base.h"

namespace wolf {

TrajectoryBase::TrajectoryBase(FrameStructure _frame_structure) :
//    NodeLinked(MID, "TRAJECTORY"),
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

        frame_list_.insert(computeFrameOrder(_frame_ptr), _frame_ptr);
//        insertDownNode(_frame_ptr, computeFrameOrder(_frame_ptr));
    }
    else
    {
        frame_list_.push_back(_frame_ptr);
        _frame_ptr->setTrajectoryPtr(this);
        addFrame(_frame_ptr);
    }

    return _frame_ptr;
}

void TrajectoryBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(auto fr_it = getFrameListPtr()->begin(); fr_it != getFrameListPtr()->end(); ++fr_it)
		(*fr_it)->getConstraintList(_ctr_list);
}

void TrajectoryBase::sortFrame(FrameBase* _frame_ptr)
{
    moveFrame(_frame_ptr, computeFrameOrder(_frame_ptr));
}

FrameBaseIter TrajectoryBase::computeFrameOrder(FrameBase* _frame_ptr)
{
    for (auto frm_rit = getFrameListPtr()->rbegin(); frm_rit != getFrameListPtr()->rend(); frm_rit++)
        if ((*frm_rit)!= _frame_ptr && (*frm_rit)->isKey() && (*frm_rit)->getTimeStamp() < _frame_ptr->getTimeStamp())
            return frm_rit.base();
    return getFrameListPtr()->begin();
}

FrameBase* TrajectoryBase::closestKeyFrameToTimeStamp(const TimeStamp& _ts)
{
    FrameBase* closest_kf = nullptr;
    Scalar min_dt = 1e9;

    for (auto frm_rit = getFrameListPtr()->rbegin(); frm_rit != getFrameListPtr()->rend(); frm_rit++)
        if ((*frm_rit)->isKey())
        {
            if (std::abs((*frm_rit)->getTimeStamp().get() - _ts.get()) < min_dt)
            {
                min_dt = std::abs((*frm_rit)->getTimeStamp().get() - _ts.get());
                closest_kf = *frm_rit;
            }
            else
                break;
        }
    return closest_kf;
}

} // namespace wolf
