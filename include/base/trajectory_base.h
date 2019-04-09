
#ifndef TRAJECTORY_BASE_H_
#define TRAJECTORY_BASE_H_

// Fwd refs
namespace wolf{
class Problem;
class FrameBase;
class TimeStamp;
}

//Wolf includes
#include "base/wolf.h"
#include "base/node_base.h"

//std includes

namespace wolf {

//class TrajectoryBase
class TrajectoryBase : public NodeBase, public std::enable_shared_from_this<TrajectoryBase>
{
    private:
        std::list<FrameBasePtr> frame_list_;

    protected:
        std::string frame_structure_;  // Defines the structure of the Frames in the Trajectory.
        FrameBasePtr last_key_frame_ptr_; // keeps pointer to the last key frame
        
    public:
        TrajectoryBase(const std::string& _frame_sturcture);
        virtual ~TrajectoryBase();
        
        // Properties
        std::string getFrameStructure() const;

        // Frames
        FrameBasePtr addFrame(FrameBasePtr _frame_ptr);
        FrameBasePtrList& getFrameList();
        FrameBasePtr getLastFrame();
        FrameBasePtr getLastKeyFrame();
        FrameBasePtr findLastKeyFrame();
        FrameBasePtr closestKeyFrameToTimeStamp(const TimeStamp& _ts);
        void setLastKeyFramePtr(FrameBasePtr _key_frame_ptr);
        void sortFrame(FrameBasePtr _frm_ptr);
        void moveFrame(FrameBasePtr _frm_ptr, FrameBaseIter _place);
        FrameBaseIter computeFrameOrder(FrameBasePtr _frame_ptr);

        // factors
        void getFactorList(FactorBasePtrList & _ctr_list);

};

inline FrameBasePtrList& TrajectoryBase::getFrameList()
{
    return frame_list_;
}

inline FrameBasePtr TrajectoryBase::getLastFrame()
{
    return frame_list_.back();
}

inline FrameBasePtr TrajectoryBase::getLastKeyFrame()
{
    return last_key_frame_ptr_;
}

inline void TrajectoryBase::setLastKeyFramePtr(FrameBasePtr _key_frame_ptr)
{
    last_key_frame_ptr_ = _key_frame_ptr;
}

inline std::string TrajectoryBase::getFrameStructure() const
{
    return frame_structure_;
}

} // namespace wolf

#endif
