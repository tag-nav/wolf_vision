
#ifndef TRAJECTORY_BASE_H_
#define TRAJECTORY_BASE_H_

// Fwd refs
namespace wolf{
class Problem;
class FrameBase;
class TimeStamp;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"

//std includes


namespace wolf {

//class TrajectoryBase
class TrajectoryBase : public NodeBase, public std::enable_shared_from_this<TrajectoryBase>
{
    private:
        std::list<FrameBasePtr> frame_list_;

    protected:
        FrameStructure frame_structure_; // Defines the structure of the Frames in the Trajectory.
        FrameBasePtr last_key_frame_ptr_;   // TODO check pointer type // keeps pointer to the last key frame
        
    public:
        TrajectoryBase(FrameStructure _frame_sturcture);
        virtual ~TrajectoryBase();
        
        // Properties
        FrameStructure getFrameStructure() const;

        // Frames
        FrameBasePtr addFrame(FrameBasePtr _frame_ptr);
//        void removeFrame(const FrameBaseIter& _frame_iter);
//        void removeFrame(const FrameBasePtr _frame_ptr);
        FrameBaseList& getFrameList();
        FrameBasePtr getLastFramePtr();
        FrameBasePtr getLastKeyFramePtr();
        FrameBasePtr closestKeyFrameToTimeStamp(const TimeStamp& _ts);
        void setLastKeyFramePtr(FrameBasePtr _key_frame_ptr);
        void sortFrame(FrameBasePtr _frm_ptr);
        void moveFrame(FrameBasePtr _frm_ptr, FrameBaseIter _place);
        FrameBaseIter computeFrameOrder(FrameBasePtr _frame_ptr);

        // constraints
        void getConstraintList(ConstraintBaseList & _ctr_list);

};

}

#include "feature_base.h"

namespace wolf{

inline void TrajectoryBase::moveFrame(FrameBasePtr _frm_ptr, FrameBaseIter _place)
{
    if (*_place != _frm_ptr)
    {
        frame_list_.remove(_frm_ptr);
        frame_list_.insert(_place, _frm_ptr);
    }
}

//inline void TrajectoryBase::removeFrame(const FrameBaseIter& _frame_iter)
//{
//    frame_list_.erase(_frame_iter);
////    delete * _frame_iter;
//}
//inline void TrajectoryBase::removeFrame(const FrameBasePtr _frame_ptr)
//{
//    frame_list_.remove(_frame_ptr);
////    delete _frame_ptr;
//}

inline FrameBaseList& TrajectoryBase::getFrameList()
{
    return frame_list_;
}

inline FrameBasePtr TrajectoryBase::getLastFramePtr()
{
    return frame_list_.back();
}

inline FrameBasePtr TrajectoryBase::getLastKeyFramePtr()
{
    return last_key_frame_ptr_;
}

inline void TrajectoryBase::setLastKeyFramePtr(FrameBasePtr _key_frame_ptr)
{
    last_key_frame_ptr_ = _key_frame_ptr;
}


inline FrameStructure TrajectoryBase::getFrameStructure() const
{
    return frame_structure_;
}


} // namespace wolf

#endif
