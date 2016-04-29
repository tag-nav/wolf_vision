
#ifndef MAP_BASE_H_
#define MAP_BASE_H_

// Fwd refs
namespace wolf{
class Problem;
class LandmarkBase;
}

//Wolf includes
#include "wolf.h"
#include "node_linked.h"

//std includes

namespace wolf {

//class MapBase
class MapBase : public NodeLinked<Problem,LandmarkBase>
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
        virtual LandmarkBase* addLandmark(LandmarkBase* _landmark_ptr);

        /** \brief Adds a landmark
         *
         * Adds a landmark to the Map. It also updates the lists of StateBlocks that are used by the solver.
         **/
        virtual void addLandmarkList(LandmarkBaseList _landmark_list);

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
