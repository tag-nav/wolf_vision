
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
class TrajectoryBase : public NodeBase //: public NodeBase // NodeLinked<Problem,FrameBase>
{
    private:
        ProblemPtr problem_ptr_;
        std::list<FrameBase*> frame_list_;

    protected:
        FrameStructure frame_structure_; // Defines the structure of the Frames in the Trajectory.
        FrameBase* last_key_frame_ptr_;  // keeps pointer to the last key frame
        
    public:
        TrajectoryBase(FrameStructure _frame_sturcture);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/        
        virtual ~TrajectoryBase();
        void destruct();
        
        /** \brief Add a frame to the trajectory
         **/
        FrameBase* addFrame(FrameBase* _frame_ptr);

        /** \brief Remove a frame to the trajectory
         **/
        void removeFrame(const FrameBaseIter& _frame_iter);
        void removeFrame(const FrameBasePtr _frame_ptr);

        /** \brief Returns a pointer to frame list
         **/
        FrameBaseList* getFrameListPtr();

        /** \brief Returns a pointer to last frame
         **/
        FrameBase* getLastFramePtr();

        /** \brief Returns a pointer to last key frame
         */
        FrameBase* getLastKeyFramePtr();

        /** \brief Sets the pointer to last key frame
         */
        void setLastKeyFramePtr(FrameBase* _key_frame_ptr);

        /** \brief Returns a list of all constraints in the trajectory thru reference
         **/
        void getConstraintList(ConstraintBaseList & _ctr_list);
        
        /** \brief Returns the frame structure (see wolf.h)
         **/
        FrameStructure getFrameStructure() const;

        /** \brief Sorts the frame by timestamp
         **/
        void sortFrame(FrameBase* _frame_iter);

        void moveFrame(FrameBasePtr _frm_ptr, FrameBaseIter _place)
        {
            if (*_place != _frm_ptr)
                {
                    frame_list_.remove(_frm_ptr);
                    frame_list_.insert(_place, _frm_ptr);
                }
        }

        /** \brief Compute the position where the frame should be
         **/
        FrameBaseIter computeFrameOrder(FrameBase* _frame_ptr);

        /** \brief Finds the closes key frame to a given timestamp
         **/
        FrameBase* closestKeyFrameToTimeStamp(const TimeStamp& _ts);

        Problem* getProblem(){return problem_ptr_;}
        void setProblem(Problem* _prob_ptr){problem_ptr_ = _prob_ptr;}

};

}

#include "feature_base.h"

namespace wolf{

inline void TrajectoryBase::removeFrame(const FrameBaseIter& _frame_iter)
{
//    removeDownNode(_frame_iter);
    frame_list_.erase(_frame_iter);
    delete * _frame_iter;
}
inline void TrajectoryBase::removeFrame(const FrameBasePtr _frame_ptr)
{
//    removeDownNode(_frame_ptr);
    frame_list_.remove(_frame_ptr);
    delete _frame_ptr;
}

inline FrameBaseList* TrajectoryBase::getFrameListPtr()
{
//    return getDownNodeListPtr();
    return & frame_list_;
}

inline FrameBase* TrajectoryBase::getLastFramePtr()
{
//    return getDownNodeListPtr()->back();
    return frame_list_.back();
}

inline FrameBase* TrajectoryBase::getLastKeyFramePtr()
{
    return last_key_frame_ptr_;
}

inline void TrajectoryBase::setLastKeyFramePtr(FrameBase* _key_frame_ptr)
{
    last_key_frame_ptr_ = _key_frame_ptr;
}

inline void TrajectoryBase ::destruct()
{
    // TODO implement something
    if (!is_deleting_)
    {
//        if (problem_ptr_ != nullptr) // && !up_node_ptr_->isTop())
//        {
//            //std::cout << "upper node is not WolfProblem " << std::endl;
//            problem_ptr_->removeTrajectory(this);
//        }
//        else
//        {
            //std::cout << "upper node is WolfProblem or nullptr" << std::endl;
            delete this;
//        }
    }
}

inline FrameStructure TrajectoryBase::getFrameStructure() const
{
    return frame_structure_;
}


} // namespace wolf

#endif
