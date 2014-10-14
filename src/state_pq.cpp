#include "state_pq.h"

using namespace Eigen;

StatePQ::StatePQ() :
        StatePose(SIZE_)
{
}

StatePQ::StatePQ(const VectorXs& _x) :
        StatePose(_x)
{
}

StatePQ::StatePQ(Vector3s& _p, Quaternions& _q) :
        StatePose(SIZE_, _p, _q)
{
}

StatePQ::StatePQ(VectorXs& _storage, unsigned int _idx) :
        StatePose(_storage, _idx, SIZE_)
{
}

StatePQ::StatePQ(VectorXs& _storage, unsigned int _idx, VectorXs& _x) :
        StatePose(_storage, _idx, _x)
{
}

StatePQ::StatePQ(VectorXs& _storage, unsigned int _idx, Vector3s& _p, Quaternions& _q) :
        StatePose(_storage, _idx, SIZE_, _p, _q)
{
}

StatePQ::~StatePQ()
{
}
