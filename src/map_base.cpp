#include "map_base.h"
//#include "problem.h"
#include "landmark_base.h"


namespace wolf {

MapBase::MapBase() :
    NodeLinked(MID, "MAP")
{
    //std::cout << "MapBase::MapBase(): " << __LINE__ << std::endl;
}

MapBase::~MapBase()
{
	//std::cout << "deleting MapBase " << nodeId() << std::endl;
}

LandmarkBase* MapBase::addLandmark(LandmarkBase* _landmark_ptr)
{
	//std::cout << "MapBase::addLandmark" << std::endl;
    addDownNode(_landmark_ptr);
    _landmark_ptr->registerNewStateBlocks();
    return _landmark_ptr;
}

void MapBase::addLandmarkList(LandmarkBaseList _landmark_list)
{
	//std::cout << "MapBase::addLandmarkList" << std::endl;
	LandmarkBaseList lmk_list_copy = _landmark_list; //since _landmark_list will be empty after addDownNodeList()
	addDownNodeList(_landmark_list);
    for (auto landmark_ptr : lmk_list_copy)
        landmark_ptr->registerNewStateBlocks();
}

void MapBase::removeLandmark(LandmarkBase* _landmark_ptr)
{
    removeDownNode(_landmark_ptr->nodeId());
}

void MapBase::removeLandmark(const LandmarkBaseIter& _landmark_iter)
{
    removeDownNode(_landmark_iter);
}

} // namespace wolf
