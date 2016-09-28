
#ifndef TRAJECTORY_BASE_H_
#define TRAJECTORY_BASE_H_

// Fwd refs
namespace wolf{
class Problem;
class FrameBase;
}

//Wolf includes
#include "wolf.h"

//std includes


namespace wolf {

//class TrajectoryBase;
//typedef TrajectoryBase* TrajectoryBasePtr;

//class TrajectoryBase
class TrajectoryBase //: public NodeBase // NodeLinked<Problem,FrameBase>
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
        void destruct()
        {
            // TODO fill code
        }
        
        /** \brief Add a frame to the trajectory
         **/
        FrameBase* addFrame(FrameBase* _frame_ptr);

        /** \brief Remove a frame to the trajectory
         **/
        void removeFrame(const FrameBaseIter& _frame_iter);

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

        /** \brief Compute the position where the frame should be
         **/
        FrameBaseIter computeFrameOrder(FrameBase* _frame_ptr);

        /** \brief Finds the closes key frame to a given timestamp
         **/
        FrameBase* closestKeyFrameToTimeStamp(const TimeStamp& _ts);

        Problem* getProblem(){return problem_ptr_;}
        void setProblem(Problem* _prob_ptr){problem_ptr_ = _prob_ptr;}

};

inline void TrajectoryBase::removeFrame(const FrameBaseIter& _frame_iter)
{
//    removeDownNode(_frame_iter);

    frame_list_.erase(_frame_iter);
    delete * _frame_iter;
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

inline FrameStructure TrajectoryBase::getFrameStructure() const
{
    return frame_structure_;
}


} // namespace wolf

#endif
