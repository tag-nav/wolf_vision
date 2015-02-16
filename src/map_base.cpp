#include "map_base.h"

MapBase::MapBase() :
    NodeLinked(TOP, "TRAJECTORY")
{
    //
}

MapBase::~MapBase()
{
    //
}

const LandmarkBaseList* MapBase::getLandmarkListPtr()
{
    return getDownNodeListPtr();
}
