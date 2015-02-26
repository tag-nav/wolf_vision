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

const LandmarkBaseList* MapBase::getLandmarkListPtr()
{
    return getDownNodeListPtr();
}
