#include "map_base.h"

MapBase::MapBase() :
    NodeLinked(MID, "TRAJECTORY")
{
    //
}

MapBase::~MapBase()
{
    //
}

void MapBase::addLandmark(LandmarkBaseShPtr& _landmark_ptr)
{
	addDownNode(_landmark_ptr);
}

void MapBase::removeLandmark(const LandmarkBaseIter& _landmark_iter)
{
	removeDownNode(_landmark_iter);
}

void MapBase::removeLandmark(const LandmarkBasePtr _landmark_ptr)
{
	removeDownNode(_landmark_ptr->nodeId());
}

const LandmarkBaseList* MapBase::getLandmarkListPtr()
{
    return getDownNodeListPtr();
}
