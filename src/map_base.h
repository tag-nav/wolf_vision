
#ifndef MAP_BASE_H_
#define MAP_BASE_H_

// Fwd refs
namespace wolf{
class Problem;
class LandmarkBase;
}

//Wolf includes
#include "wolf.h"
#include "node_base.h"

//std includes

namespace wolf {

//class MapBase
class MapBase : public NodeBase
{
    private:
        ProblemWPtr problem_ptr_;
        LandmarkBaseList landmark_list_;

    public:
        MapBase();

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         **/        
        ~MapBase();
        void destruct();
        
        ProblemPtr getProblem(){return problem_ptr_;}
        void setProblem(ProblemPtr _prob_ptr){problem_ptr_ = _prob_ptr;}

        virtual LandmarkBasePtr addLandmark(LandmarkBasePtr _landmark_ptr);
        virtual void addLandmarkList(LandmarkBaseList _landmark_list);
        void removeLandmark(const LandmarkBaseIter& _landmark_iter);
        void removeLandmark(LandmarkBasePtr _landmark_ptr);
        LandmarkBaseList* getLandmarkListPtr();
        
        void load(const std::string& _map_file_yaml);
        void save(const std::string& _map_file_yaml, const std::string& _map_name = "Map automatically saved by Wolf");

    private:
        std::string dateTimeNow();
};

inline void MapBase::destruct()
{
    if (!is_deleting_)
        delete this;
}

inline LandmarkBaseList* MapBase::getLandmarkListPtr()
{
    return & landmark_list_;
}

} // namespace wolf

#endif
