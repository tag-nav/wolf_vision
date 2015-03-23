#include "map_base.h"

MapBase::MapBase() :
    NodeLinked(MID, "MAP")
{
    //std::cout << "MapBase::MapBase(): " << __LINE__ << std::endl;
}

MapBase::~MapBase()
{
	//std::cout << "deleting MapBase " << nodeId() << std::endl;
}

void MapBase::addLandmark(LandmarkBase* _landmark_ptr)
{
	addDownNode(_landmark_ptr);
}

void MapBase::removeLandmark(const LandmarkBaseIter& _landmark_iter)
{
	removeDownNode(_landmark_iter);
}

void MapBase::removeLandmark(LandmarkBase* _landmark_ptr)
{
	removeDownNode(_landmark_ptr->nodeId());
}

LandmarkBaseList* MapBase::getLandmarkListPtr()
{
    return getDownNodeListPtr();
}
