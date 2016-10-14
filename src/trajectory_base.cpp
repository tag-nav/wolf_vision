#include "trajectory_base.h"
#include "frame_base.h"

namespace wolf {

TrajectoryBase::TrajectoryBase(FrameStructure _frame_structure) :
    NodeBase("TRAJECTORY"),
    problem_ptr_(nullptr),
    frame_structure_(_frame_structure), last_key_frame_ptr_(nullptr)
{
    //
}

TrajectoryBase::~TrajectoryBase()
{
    is_removing_ = true;
    while (!frame_list_.empty())
    {
        delete frame_list_.front();
        frame_list_.pop_front();
    }
    //std::cout << "deleting TrajectoryBase " << nodeId() << std::endl;
}

void TrajectoryBase::destruct()
{
    if (!is_removing_)
        delete this;
}

FrameBasePtr TrajectoryBase::addFrame(FrameBasePtr _frame_ptr)
{
    // link up
    _frame_ptr->setTrajectoryPtr(this);
    _frame_ptr->setProblem(getProblem());

    if (_frame_ptr->isKey())
    {
        _frame_ptr->registerNewStateBlocks();
        if (last_key_frame_ptr_ == nullptr || last_key_frame_ptr_->getTimeStamp() < _frame_ptr->getTimeStamp())
            last_key_frame_ptr_ = _frame_ptr;

        frame_list_.insert(computeFrameOrder(_frame_ptr), _frame_ptr);
    }
    else
    {
        //        addFrame(_frame_ptr);
        frame_list_.push_back(_frame_ptr);
    }

    return _frame_ptr;
}


void TrajectoryBase::getConstraintList(ConstraintBaseList & _ctr_list)
{
	for(auto fr_ptr : *getFrameListPtr())
		fr_ptr->getConstraintList(_ctr_list);
}

void TrajectoryBase::sortFrame(FrameBasePtr _frame_ptr)
{
    moveFrame(_frame_ptr, computeFrameOrder(_frame_ptr));
}

FrameBaseIter TrajectoryBase::computeFrameOrder(FrameBasePtr _frame_ptr)
{
    for (auto frm_rit = getFrameListPtr()->rbegin(); frm_rit != getFrameListPtr()->rend(); frm_rit++)
        if ((*frm_rit)!= _frame_ptr && (*frm_rit)->isKey() && (*frm_rit)->getTimeStamp() < _frame_ptr->getTimeStamp())
            return frm_rit.base();
    return getFrameListPtr()->begin();
}

FrameBasePtr TrajectoryBase::closestKeyFrameToTimeStamp(const TimeStamp& _ts)
{
    FrameBasePtr closest_kf = nullptr;
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
