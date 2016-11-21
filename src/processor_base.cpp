#include "processor_base.h"
#include "capture_base.h"
#include "frame_base.h"

namespace wolf {

unsigned int ProcessorBase::processor_id_count_ = 0;

ProcessorBase::ProcessorBase(const std::string& _type, const Scalar& _time_tolerance) :
        NodeBase("PROCESSOR"),
        sensor_ptr_(),
        processor_id_(++processor_id_count_),
        time_tolerance_(_time_tolerance)
{
    setType(_type);

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

FrameBasePtr ProcessorBase::emplaceFrame(CaptureBasePtr _capture_ptr, FrameType _type)
{
    std::cout << "Making " << (_type == KEY_FRAME? "key-" : "") << "frame" << std::endl;

    FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(_type, _capture_ptr->getTimeStamp());
    new_frame_ptr->addCapture(_capture_ptr);

    return new_frame_ptr;
}

FrameBasePtr ProcessorBase::emplaceFrame(CaptureBasePtr _capture_ptr, const Eigen::VectorXs& _state, FrameType _type)
{
    std::cout << "Making " << (_type == KEY_FRAME? "key-" : "") << "frame" << std::endl;

    FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(_type, _state, _capture_ptr->getTimeStamp());
    new_frame_ptr->addCapture(_capture_ptr);

    return new_frame_ptr;
}

void ProcessorBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
//        std::cout << "Removing     p" << id() << std::endl;
        ProcessorBasePtr this_p = shared_from_this();

        // remove from upstream
        SensorBasePtr sen = sensor_ptr_.lock();
        if(sen)
            sen->getProcessorList().remove(this_p);
    }
}

} // namespace wolf
