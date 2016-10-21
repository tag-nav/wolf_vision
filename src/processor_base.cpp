#include "processor_base.h"
#include "capture_base.h"
#include "frame_base.h"

namespace wolf {

unsigned int ProcessorBase::processor_id_count_ = 0;

ProcessorBase::ProcessorBase(ProcessorType _tp, const std::string& _type, const Scalar& _time_tolerance) :
        NodeBase("PROCESSOR"),
        sensor_ptr_(),
        processor_id_(++processor_id_count_),
        type_id_(_tp),
        time_tolerance_(_time_tolerance)
{
    std::cout << "constructed    +p" << id() << std::endl;

    //
}

ProcessorBase::~ProcessorBase()
{
//    remove();
    std::cout << "destructed     -p" << id() << std::endl;
}

bool ProcessorBase::permittedKeyFrame()
{
    return getProblem()->permitKeyFrame(shared_from_this());
}

void ProcessorBase::makeFrame(CaptureBasePtr _capture_ptr, FrameKeyType _type)
{
    std::cout << __FILE__ << ":" << __FUNCTION__ << "():" << __LINE__ << std::endl;

    // We need to create the new free Frame to hold what will become the last Capture
    FrameBasePtr new_frame_ptr = getProblem()->createFrame(_type, _capture_ptr->getTimeStamp());

    new_frame_ptr->addCapture(_capture_ptr); // Add incoming Capture to the new Frame

    if (_type == KEY_FRAME)
        // Keyframe callback in order to let the other processors to establish their constraints
        getProblem()->keyFrameCallback(_capture_ptr->getFramePtr(), shared_from_this(), time_tolerance_);

    std::cout << __FILE__ << ":" << __FUNCTION__ << "():" << __LINE__ << std::endl;
}

void ProcessorBase::remove()
{
    if (!is_removing_)
    {
        is_removing_ = true;
        std::cout << "Removing     p" << id() << std::endl;
        ProcessorBasePtr this_p = shared_from_this();

        // remove from upstream
        SensorBasePtr sen = sensor_ptr_.lock();
        if(sen)
            sen->getProcessorList().remove(this_p);
    }
}

} // namespace wolf
