
#ifndef TRAJECTORY_BASE_H_
#define TRAJECTORY_BASE_H_

class NodeTerminus;
class FrameBase;

//std includes
#include <iostream>
#include <vector>
#include <list>
#include <random>
#include <cmath>

//Wolf includes
#include "wolf.h"
#include "node_linked.h"
#include "node_terminus.h"
#include "frame_base.h"
#include "state_base.h"

//class TrajectoryBase
class TrajectoryBase : public NodeLinked<NodeTerminus,FrameBase>
{
    protected:
		// JVN: No seria millor que la trajectòria de tamany fix fos una classe derivada? i implementar funcions virtuals de l'estil "manageFrames()" que elimini/remapegi
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
        void addFrame(FrameBaseShPtr& _frame_ptr);

        /** \brief Returns a pointer to Frame list
         * 
         * Returns a pointer to Frame list
         * 
         **/
        FrameBaseList* getFrameListPtr();

        /** \brief Returns a const pointer to Frame list
         * 
         * Returns a const pointer to Frame list
         * 
         **/
//         const FrameBaseList* frameList() const;
        
        /** \brief Prints self info to std out
         *
         * Prints self info to std out
         *
         **/        
        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const {};
        
};
#endif
