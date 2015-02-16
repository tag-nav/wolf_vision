
#ifndef MAP_BASE_H_
#define MAP_BASE_H_

class NodeTerminus;
class LandmarkBase;

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
#include "landmark_base.h"

//class MapBase
class MapBase : public NodeLinked<NodeTerminus,LandmarkBase>
{
    protected:

        
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
