#include "base/processor/processor_base.h"
#include "base/processor/processor_motion.h"
#include "base/capture/capture_base.h"
#include "base/frame/frame_base.h"

namespace wolf {

unsigned int ProcessorBase::processor_id_count_ = 0;

ProcessorBase::ProcessorBase(const std::string& _type, ProcessorParamsBasePtr _params) :
        NodeBase("PROCESSOR", _type),
        processor_id_(++processor_id_count_),
        params_(_params),
        sensor_ptr_()
{
//    WOLF_DEBUG("constructed    +p" , id());
}

ProcessorBase::~ProcessorBase()
{
//    WOLF_DEBUG("destructed     -p" , id());
}

bool ProcessorBase::permittedKeyFrame()
{
    return isVotingActive() && getProblem()->permitKeyFrame(shared_from_this());
}

FrameBasePtr ProcessorBase::emplaceFrame(FrameType _type, CaptureBasePtr _capture_ptr)
{
    std::cout << "Making " << (_type == KEY_FRAME? "key-" : "") << "frame" << std::endl;

    FrameBasePtr new_frame_ptr = getProblem()->emplaceFrame(_type, _capture_ptr->getTimeStamp());
    // new_frame_ptr->addCapture(_capture_ptr);
    _capture_ptr->link(new_frame_ptr);

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
    WOLF_DEBUG("P", isMotion() ? "M " : "T ", getName(), ": KF", _keyframe_ptr->id(), " callback received with ts = ", _keyframe_ptr->getTimeStamp());
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
            if(P && P->getProcessorMotion()->id() == this->id())
                P->clearProcessorMotion();
        }

        // remove from upstream
        SensorBasePtr sen = sensor_ptr_.lock();
        if(sen)
            sen->getProcessorList().remove(this_p);
    }
}
    void ProcessorBase::link(SensorBasePtr _sen_ptr)
    {
        std::cout << "Linking ProcessorBase" << std::endl;
        _sen_ptr->addProcessor(shared_from_this());
        this->setSensor(_sen_ptr);
        this->setProblem(_sen_ptr->getProblem());
    }
/////////////////////////////////////////////////////////////////////////////////////////

void PackKeyFrameBuffer::removeUpTo(const TimeStamp& _time_stamp)
{
    PackKeyFrameBuffer::Iterator post = container_.upper_bound(_time_stamp);
    container_.erase(container_.begin(), post); // erasing by range
}

void PackKeyFrameBuffer::add(const FrameBasePtr& _key_frame, const Scalar& _time_tolerance)
{
    TimeStamp time_stamp = _key_frame->getTimeStamp();
    PackKeyFramePtr kfpack = std::make_shared<PackKeyFrame>(_key_frame, _time_tolerance);
    container_.emplace(time_stamp, kfpack);
}

PackKeyFramePtr PackKeyFrameBuffer::selectPack(const TimeStamp& _time_stamp, const Scalar& _time_tolerance)
{
    if (container_.empty())
        return nullptr;

    PackKeyFrameBuffer::Iterator post = container_.upper_bound(_time_stamp);

    // remove packs corresponding to removed KFs (keeping the next iterator in post)
    while (post != container_.end() && post->second->key_frame->isRemoving())
        post = container_.erase(post);
    while (post != container_.begin() && std::prev(post)->second->key_frame->isRemoving())
        container_.erase(std::prev(post));

    bool prev_exists = (post != container_.begin());
    bool post_exists = (post != container_.end());

    bool post_ok = post_exists && checkTimeTolerance(post->first, post->second->time_tolerance, _time_stamp, _time_tolerance);

    if (prev_exists)
    {
        PackKeyFrameBuffer::Iterator prev = std::prev(post);

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
PackKeyFramePtr PackKeyFrameBuffer::selectPack(const CaptureBasePtr _capture, const Scalar& _time_tolerance)
{
    return selectPack(_capture->getTimeStamp(), _time_tolerance);
}

PackKeyFramePtr PackKeyFrameBuffer::selectFirstPackBefore(const TimeStamp& _time_stamp, const Scalar& _time_tolerance)
{
    // remove packs corresponding to removed KFs
    while (!container_.empty() && container_.begin()->second->key_frame->isRemoving())
        container_.erase(container_.begin());

    // There is no pack
    if (container_.empty())
         return nullptr;

    // Checking on begin() since packs are ordered in time
    // Return first pack if is older than time stamp
    if (container_.begin()->first < _time_stamp)
         return container_.begin()->second;

    // Return first pack if despite being newer, it is within the time tolerance
    if (checkTimeTolerance(container_.begin()->first, container_.begin()->second->time_tolerance, _time_stamp, _time_tolerance))
        return container_.begin()->second;

    // otherwise return nullptr (no pack before the provided ts or within the tolerance was found)
    return nullptr;

}

PackKeyFramePtr PackKeyFrameBuffer::selectFirstPackBefore(const CaptureBasePtr _capture, const Scalar& _time_tolerance)
{
    return selectFirstPackBefore(_capture->getTimeStamp(), _time_tolerance);
}

void PackKeyFrameBuffer::print(void)
{
    std::cout << "[ ";
    for (auto iter : container_)
    {
        std::cout << "( tstamp: " << iter.first << ", id: " << iter.second->key_frame->id() << ") ";
    }
    std::cout << "]" << std::endl;
}

bool PackKeyFrameBuffer::checkTimeTolerance(const TimeStamp& _time_stamp1, const Scalar& _time_tolerance1,
                                      const TimeStamp& _time_stamp2, const Scalar& _time_tolerance2)
{
    Scalar time_diff = std::fabs(_time_stamp1 - _time_stamp2);
    Scalar time_tol  = std::min(_time_tolerance1, _time_tolerance2);
    bool pass = time_diff <= time_tol;
    return pass;
}
} // namespace wolf
