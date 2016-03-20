#include "local_parametrization_base.h"

LocalParametrizationBase::LocalParametrizationBase(unsigned int _global_size, unsigned int _local_size) :
        global_size_(_global_size), local_size_(_local_size)
{
}

LocalParametrizationBase::~LocalParametrizationBase()
{
}

unsigned int LocalParametrizationBase::getLocalSize() const
{
    return local_size_;
}

unsigned int LocalParametrizationBase::getGlobalSize() const
{
    return global_size_;
}
