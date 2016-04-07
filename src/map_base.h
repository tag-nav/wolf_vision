
#ifndef MAP_BASE_H_
#define MAP_BASE_H_

// Fwd refs
namespace wolf{
class WolfProblem;
class LandmarkBase;
}

//Wolf includes
#include "wolf.h"
#include "node_linked.h"

//std includes

namespace wolf {

//class MapBase
class MapBase : public NodeLinked<WolfProblem,LandmarkBase>
{
    public:
        MapBase();

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         **/        
        ~MapBase();
        
        /** \brief Adds a landmark
         *
         * Adds a landmark to the Map. It also updates the lists of StateBlocks that are used by the solver.
         **/
        virtual void addLandmark(LandmarkBase* _landmark_ptr);

        void removeLandmark(const LandmarkBaseIter& _landmark_iter);
        void removeLandmark(LandmarkBase* _landmark_ptr);

        LandmarkBaseList* getLandmarkListPtr();
        

};

inline LandmarkBaseList* MapBase::getLandmarkListPtr()
{
    return getDownNodeListPtr();
}

} // namespace wolf

#endif
