
#ifndef TRAJECTORY_BASE_H_
#define TRAJECTORY_BASE_H_

// Fwd refs
namespace wolf{
class Problem;
class FrameBase;
class TimeStamp;
}

//Wolf includes
#include "base/common/wolf.h"
#include "base/common/node_base.h"

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
        FrameBasePtr last_estimated_frame_ptr_; // keeps pointer to the last estimated frame
        
    public:
        TrajectoryBase(const std::string& _frame_sturcture);
        virtual ~TrajectoryBase();
        
        // Properties
        std::string getFrameStructure() const;

        // Frames
        FrameBasePtr addFrame(FrameBasePtr _frame_ptr);
        FrameBasePtrList& getFrameList();
        const FrameBasePtrList& getFrameList() const;
        FrameBasePtr getLastFrame() const;
        FrameBasePtr getLastKeyFrame() const;
        FrameBasePtr getLastEstimatedFrame() const;
        FrameBasePtr closestKeyFrameToTimeStamp(const TimeStamp& _ts) const;
        FrameBasePtr closestEstimatedFrameToTimeStamp(const TimeStamp& _ts) const;
        void sortFrame(FrameBasePtr _frm_ptr);
        void updateLastFrames();

        // factors
        void getFactorList(FactorBasePtrList & _fac_list);

    protected:
        FrameBaseIter computeFrameOrder(FrameBasePtr _frame_ptr);
        void moveFrame(FrameBasePtr _frm_ptr, FrameBaseIter _place);
};

inline FrameBasePtrList& TrajectoryBase::getFrameList()
{
    return frame_list_;
}

inline const FrameBasePtrList& TrajectoryBase::getFrameList() const
{
    return frame_list_;
}

inline FrameBasePtr TrajectoryBase::getLastFrame() const
{
    return frame_list_.back();
}

inline FrameBasePtr TrajectoryBase::getLastKeyFrame() const
{
    return last_key_frame_ptr_;
}

inline FrameBasePtr TrajectoryBase::getLastEstimatedFrame() const
{
    return last_estimated_frame_ptr_;
}

inline std::string TrajectoryBase::getFrameStructure() const
{
    return frame_structure_;
}

} // namespace wolf

#endif
