
#ifndef TRAJECTORY_BASE_H_
#define TRAJECTORY_BASE_H_

class WolfProblem;
class FrameBase;

//std includes
#include <iostream>
#include <vector>
#include <list>
#include <random>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "wolf_problem.h"
#include "node_linked.h"
#include "node_terminus.h"
#include "frame_base.h"
#include "state_base.h"

//class TrajectoryBase
class TrajectoryBase : public NodeLinked<WolfProblem,FrameBase>
{
    protected:
		// TODO: JVN: No seria millor que això ho tingui el wolf_problem o el wolf_manager?
        unsigned int fixed_size_; // Limits the number of frames forming the trajectory
        
    public:
        /** \brief Constructor
         *
         * Constructor
         *
         **/
        TrajectoryBase();

        /** \brief Destructor
         *
         * Destructor
         *
         **/        
        ~TrajectoryBase();
        
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
        
};
#endif
