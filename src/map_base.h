
#ifndef MAP_BASE_H_
#define MAP_BASE_H_

class WolfProblem;
class LandmarkBase;

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
#include "landmark_base.h"

//class MapBase
class MapBase : public NodeLinked<WolfProblem,LandmarkBase>
{
    public:
        /** \brief Constructor
         *
         * Constructor
         *
         **/
		MapBase();

        /** \brief Destructor
         *
         * Destructor
         *
         **/        
        ~MapBase();
        
        /** \brief Adds a landmark
		 *
		 * Adds a landmark
		 *
		 **/
        void addLandmark(LandmarkBaseShPtr& _landmark_ptr);

        /** \brief Removes a landmark
		 *
		 * Removes a landmark
		 *
		 **/
        void removeLandmark(const LandmarkBaseIter& _landmark_iter);

        /** \brief Removes a landmark
		 *
		 * Removes a landmark
		 *
		 **/
        void removeLandmark(const LandmarkBasePtr _landmark_ptr);

        /** \brief Returns Frame list
         * 
         * Returns FrameBase list
         * 
         **/
        const LandmarkBaseList* getLandmarkListPtr();

        /** \brief Prints self info to std out
         *
         * Prints self info to std out
         *
         **/        
        virtual void printSelf(unsigned int _ntabs = 0, std::ostream& _ost = std::cout) const {};
        
};
#endif
