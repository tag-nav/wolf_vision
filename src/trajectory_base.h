
#ifndef TRAJECTORY_BASE_H_
#define TRAJECTORY_BASE_H_

// Fwd refs
class WolfProblem;
class FrameBase;

//Wolf includes
#include "wolf.h"
#include "node_linked.h"

//std includes

//class TrajectoryBase
class TrajectoryBase : public NodeLinked<WolfProblem,FrameBase>
{
    protected:
        FrameStructure frame_structure_; // Defines the structure of the Frames in the Trajectory.
        // TODO: JVN: No seria millor que aixo ho tingui el wolf_problem o el wolf_manager?
        unsigned int fixed_size_; // Limits the number of Frames forming the Trajectory
        
    public:
        /** \brief Constructor
         *
         * Constructor
         *
         **/
        TrajectoryBase(FrameStructure _frame_sturcture);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/        
        virtual ~TrajectoryBase();
        
        /** \brief Add a frame to the trajectory
         *
         * Add a frame to the trajectory
         *
         **/
        void addFrame(FrameBase* _frame_ptr);

        /** \brief Remove a frame to the trajectory
         *
         * Remove a frame to the trajectory
         *
         **/
        void removeFrame(const FrameBaseIter& _frame_ptr);

        /** \brief Returns a pointer to Frame list
         * 
         * Returns a pointer to Frame list
         * 
         **/
        FrameBaseList* getFrameListPtr();

        /** \brief Returns a pointer to last Frame
         *
         * Returns a pointer to last Frame
         *
         **/
        FrameBase* getLastFramePtr();


        /** \brief Returns a list of all constraints in the trajectory thru reference
         *
         * Returns a list of all constraints in the trajectory thru reference
         *
         **/
        void getConstraintList(ConstraintBaseList & _ctr_list);
        
        FrameStructure getFrameStructure() const;
};
#endif
