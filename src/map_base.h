
#ifndef MAP_BASE_H_
#define MAP_BASE_H_

// Fwd refs
class WolfProblem;
class LandmarkBase;

//Wolf includes
#include "wolf.h"
#include "node_linked.h"

//std includes
#include <iostream>
#include <vector>
#include <list>
#include <random>
#include <cmath>

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

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/        
        ~MapBase();
        
        /** \brief Adds a landmark
		 *
		 * Adds a landmark
		 *
		 **/
        virtual void addLandmark(LandmarkBase* _landmark_ptr);

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
        void removeLandmark(LandmarkBase* _landmark_ptr);

        /** \brief Returns landmarks list
         * 
         * Returns LandmarkBase list
         * 
         **/
        LandmarkBaseList* getLandmarkListPtr();
        
};
#endif
