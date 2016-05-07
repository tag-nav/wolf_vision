#include "processor_base.h"

namespace wolf {

unsigned int ProcessorBase::processor_id_count_ = 0;

ProcessorBase::ProcessorBase(ProcessorType _tp) :
        NodeLinked(MID, "PROCESSOR"),
        processor_id_(++processor_id_count_),
        type_id_(_tp)
{
    //
}

ProcessorBase::~ProcessorBase()
{
    //
}

bool ProcessorBase::permittedKeyFrame()
{
    return getProblem()->permitKeyFrame(this);
}

} // namespace wolf
