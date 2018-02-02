#include "processor_base.h"
#include "processor_motion.h"
#include "capture_base.h"
#include "frame_base.h"

namespace wolf {

unsigned int ProcessorBase::processor_id_count_ = 0;

ProcessorBase::ProcessorBase(const std::string& _type, const Scalar& _time_tolerance) :
        NodeBase("PROCESSOR", _type),
        sensor_ptr_(),
        is_removing_(false),
        processor_id_(++processor_id_count_),
        time_tolerance_(_time_tolerance)
{
//    WOLF_DEBUG("constructed    +p" , id());
}

ProcessorBase::~ProcessorBase()
{
//    WOLF_DEBUG("destructed     -p" , id());
}

bool ProcessorBase::permittedKeyFrame()
{
    return getProblem()->permitKeyFrame(shared_from_this());
}

FrameBasePtr ProcessorBase::emplaceFrame(FrameType _type, CaptureBasePtr _capture_ptr)
{
    std::cout << "Making " << (_type == KEY_FRAME? "key-" : "") << "frame" << std::endl;

    FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(_type, _capture_ptr->getTimeStamp());
    new_frame_ptr->addCapture(_capture_ptr);

    return new_frame_ptr;
}

FrameBasePtr ProcessorBase::emplaceFrame(FrameType _type, CaptureBasePtr _capture_ptr, const Eigen::VectorXs& _state)
{
    FrameBasePtr new_frame_ptr = emplaceFrame(_type, _capture_ptr);
    new_frame_ptr->setState(_state);

    return new_frame_ptr;
}

void ProcessorBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        ProcessorBasePtr this_p = shared_from_this();

        // clear Problem::processor_motion_ptr_
        if (isMotion())
        {
            ProblemPtr P = getProblem();
            if(P && P->getProcessorMotionPtr()->id() == this->id())
                P->clearProcessorMotion();
        }

        // remove from upstream
        SensorBasePtr sen = sensor_ptr_.lock();
        if(sen)
            sen->getProcessorList().remove(this_p);
    }
}

} // namespace wolf
