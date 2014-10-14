

#include "density_base.h"

DensityBase::DensityBase(const unsigned int _size)
{
}

DensityBase::~DensityBase()
{
}

VectorXs& DensityBase::mean()
{
    return mean_;
}
