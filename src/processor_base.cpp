#include "processor_base.h"
#include "processor_motion.h"
#include "capture_base.h"
#include "frame_base.h"

namespace wolf {

unsigned int ProcessorBase::processor_id_count_ = 0;

ProcessorBase::ProcessorBase(const std::string& _type, const Scalar& _time_tolerance) :
        NodeBase("PROCESSOR", _type),
        processor_id_(++processor_id_count_),
        time_tolerance_(_time_tolerance),
        sensor_ptr_(),
        is_removing_(false)
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

void ProcessorBase::keyFrameCallback(FrameBasePtr _keyframe_ptr, const Scalar& _time_tol_other)
{
    if (_keyframe_ptr != nullptr)
        kf_pack_buffer_.add(_keyframe_ptr,_time_tol_other);

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

/////////////////////////////////////////////////////////////////////////////////////////

void KFPackBuffer::removeUpTo(const TimeStamp& _time_stamp)
{
    KFPackBuffer::Iterator post = container_.upper_bound(_time_stamp);
    container_.erase(container_.begin(), post); // erasing by range
}

void KFPackBuffer::add(const FrameBasePtr& _key_frame, const Scalar& _time_tolerance)
{
    TimeStamp time_stamp = _key_frame->getTimeStamp();
    KFPackPtr kfpack = std::make_shared<KFPack>(_key_frame, _time_tolerance);
    container_.emplace(time_stamp, kfpack);
}

KFPackPtr KFPackBuffer::selectPack(const TimeStamp& _time_stamp, const Scalar& _time_tolerance)
{
    if (container_.empty())
        return nullptr;

    KFPackBuffer::Iterator post = container_.upper_bound(_time_stamp);

    bool prev_exists = (post != container_.begin());
    bool post_exists = (post != container_.end());

    bool post_ok = post_exists && checkTimeTolerance(post->first, post->second->time_tolerance, _time_stamp, _time_tolerance);

    if (prev_exists)
    {
        KFPackBuffer::Iterator prev = std::prev(post);

        bool prev_ok = checkTimeTolerance(prev->first, prev->second->time_tolerance, _time_stamp, _time_tolerance);

        if (prev_ok && !post_ok)
            return prev->second;

        else if (!prev_ok && post_ok)
            return post->second;

        else if (prev_ok && post_ok)
        {
            if (std::fabs(post->first - _time_stamp) < std::fabs(prev->first - _time_stamp))
                return post->second;
            else
                return prev->second;
        }
    }
    else if (post_ok)
        return post->second;

    return nullptr;
}

void KFPackBuffer::print(void)
{
    std::cout << "[ ";
    for (auto iter : container_)
    {
        std::cout << "( tstamp: " << iter.first << ", id: " << iter.second->key_frame->id() << ") ";
    }
    std::cout << "]" << std::endl;
}

bool KFPackBuffer::checkTimeTolerance(const TimeStamp& _time_stamp1, const Scalar& _time_tolerance1,
                                      const TimeStamp& _time_stamp2, const Scalar& _time_tolerance2)
{
    Scalar time_diff = std::fabs(_time_stamp1 - _time_stamp2);
    Scalar time_tol  = std::min(_time_tolerance1, _time_tolerance2);
    bool pass = time_diff <= time_tol;
    return pass;
    return (std::fabs(_time_stamp1 - _time_stamp2) <= std::min(_time_tolerance1, _time_tolerance2));
}

} // namespace wolf
