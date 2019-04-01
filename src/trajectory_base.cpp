#include "base/trajectory_base.h"
#include "base/frame_base.h"

namespace wolf {

TrajectoryBase::TrajectoryBase(const std::string& _frame_structure) :
    NodeBase("TRAJECTORY", "Base"),
    frame_structure_(_frame_structure),
    last_key_frame_ptr_(nullptr)
{
//    WOLF_DEBUG("constructed T");
}

TrajectoryBase::~TrajectoryBase()
{
    //
}

FrameBasePtr TrajectoryBase::addFrame(FrameBasePtr _frame_ptr)
{
    // link up
    _frame_ptr->setTrajectoryPtr(shared_from_this());
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
        frame_list_.push_back(_frame_ptr);
    }

    return _frame_ptr;
}

void TrajectoryBase::getFactorList(FactorBasePtrList & _fac_list)
{
	for(auto fr_ptr : getFrameList())
		fr_ptr->getFactorList(_fac_list);
}

void TrajectoryBase::sortFrame(FrameBasePtr _frame_ptr)
{
    moveFrame(_frame_ptr, computeFrameOrder(_frame_ptr));
    //    last_key_frame_ptr_ = findLastKeyFrame(); // done in moveFrame() just above
}

void TrajectoryBase::moveFrame(FrameBasePtr _frm_ptr, FrameBaseIter _place)
{
    if (*_place != _frm_ptr)
    {
        frame_list_.remove(_frm_ptr);
        frame_list_.insert(_place, _frm_ptr);
        last_key_frame_ptr_ = findLastKeyFrame();
    }
}

FrameBaseIter TrajectoryBase::computeFrameOrder(FrameBasePtr _frame_ptr)
{
    for (auto frm_rit = getFrameList().rbegin(); frm_rit != getFrameList().rend(); frm_rit++)
        if ((*frm_rit)!= _frame_ptr && (*frm_rit)->isKey() && (*frm_rit)->getTimeStamp() <= _frame_ptr->getTimeStamp())
            return frm_rit.base();
    return getFrameList().begin();
}

FrameBasePtr TrajectoryBase::findLastKeyFrame()
{
    // NOTE: Assumes keyframes are sorted by timestamp
    for (auto frm_rit = getFrameList().rbegin(); frm_rit != getFrameList().rend(); ++frm_rit)
        if ((*frm_rit)->isKey())
            return (*frm_rit);

    return nullptr;
}

FrameBasePtr TrajectoryBase::closestKeyFrameToTimeStamp(const TimeStamp& _ts)
{
    FrameBasePtr closest_kf = nullptr;
    Scalar min_dt = 1e9;

    for (auto frm_rit = getFrameList().rbegin(); frm_rit != getFrameList().rend(); frm_rit++)
        if ((*frm_rit)->isKey())
        {
            Scalar dt = std::fabs((*frm_rit)->getTimeStamp() - _ts);
            if (dt < min_dt)
            {
                min_dt = dt;
                closest_kf = *frm_rit;
            }
            else
                break;
        }
    return closest_kf;
}

} // namespace wolf
