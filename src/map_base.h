
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
class MapBase : public NodeBase, public std::enable_shared_from_this<MapBase>
{
    private:
        LandmarkBaseList landmark_list_;

    public:
        MapBase();
        ~MapBase();
        
        virtual LandmarkBasePtr addLandmark(LandmarkBasePtr _landmark_ptr);
        virtual void addLandmarkList(LandmarkBaseList& _landmark_list);
        LandmarkBaseList& getLandmarkList();
        
        void load(const std::string& _map_file_yaml);
        void save(const std::string& _map_file_yaml, const std::string& _map_name = "Map automatically saved by Wolf");

    private:
        std::string dateTimeNow();
};

inline LandmarkBaseList& MapBase::getLandmarkList()
{
    return landmark_list_;
}

} // namespace wolf

#endif
