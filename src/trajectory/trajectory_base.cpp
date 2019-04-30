#include "base/trajectory/trajectory_base.h"
#include "base/frame/frame_base.h"

namespace wolf {

TrajectoryBase::TrajectoryBase(const std::string& _frame_structure) :
    NodeBase("TRAJECTORY", "Base"),
    frame_structure_(_frame_structure),
    last_key_frame_ptr_(nullptr),
    last_estimated_frame_ptr_(nullptr)
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
    _frame_ptr->setTrajectory(shared_from_this());
    _frame_ptr->setProblem(getProblem());

    // add to list
    frame_list_.push_back(_frame_ptr);

    if (_frame_ptr->isEstimated())
    {
        // register state blocks
        _frame_ptr->registerNewStateBlocks();

        // sort
        sortFrame(_frame_ptr);

        // update last_estimated and last_key
        updateLastFrames();
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
}

void TrajectoryBase::moveFrame(FrameBasePtr _frm_ptr, FrameBaseIter _place)
{
    if (*_place != _frm_ptr)
    {
        frame_list_.remove(_frm_ptr);
        frame_list_.insert(_place, _frm_ptr);
        updateLastFrames();
    }
}

FrameBaseIter TrajectoryBase::computeFrameOrder(FrameBasePtr _frame_ptr)
{
    for (auto frm_rit = getFrameList().rbegin(); frm_rit != getFrameList().rend(); frm_rit++)
        if ((*frm_rit)!= _frame_ptr && (*frm_rit)->isEstimated() && (*frm_rit)->getTimeStamp() <= _frame_ptr->getTimeStamp())
            return frm_rit.base();
    return getFrameList().begin();
}

void TrajectoryBase::updateLastFrames()
{
    bool last_estimated_set = false;

    // NOTE: Assumes estimated (key or auxiliary) frames are sorted by timestamp
    for (auto frm_rit = getFrameList().rbegin(); frm_rit != getFrameList().rend(); ++frm_rit)
        if ((*frm_rit)->isEstimated())
        {
            if (!last_estimated_set)
            {
                last_estimated_frame_ptr_ = (*frm_rit);
                last_estimated_set = true;
            }
            if ((*frm_rit)->isKey())
            {
                last_key_frame_ptr_ = (*frm_rit);
                break;
            }
        }
}

FrameBasePtr TrajectoryBase::closestKeyFrameToTimeStamp(const TimeStamp& _ts) const
{
    // NOTE: Assumes estimated (key or auxiliary) frames are sorted by timestamp
    FrameBasePtr closest_kf = nullptr;
    Scalar min_dt = 1e9;

    for (auto frm_rit = frame_list_.rbegin(); frm_rit != frame_list_.rend(); frm_rit++)
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

FrameBasePtr TrajectoryBase::closestEstimatedFrameToTimeStamp(const TimeStamp& _ts) const
{
    // NOTE: Assumes estimated (key or auxiliary) frames are sorted by timestamp
    FrameBasePtr closest_kf = nullptr;
    Scalar min_dt = 1e9;

    for (auto frm_rit = frame_list_.rbegin(); frm_rit != frame_list_.rend(); frm_rit++)
        if ((*frm_rit)->isEstimated())
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
