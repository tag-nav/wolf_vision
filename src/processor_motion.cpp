#include "processor_motion.h"

ProcessorMotion::ProcessorMotion(ProcessorType _tp) : ProcessorBase(_tp)
{
}

ProcessorMotion::~ProcessorMotion()
{
}

void ProcessorMotion::composeDeltaState(const TimeStamp& _t_start, TimeStamp& _t_end, Eigen::VectorXs& _Dx)
{
    // Get time index
    i_start_ = (_t_start - ts_origin_) / dt_;
    i_end_ = (_t_end - ts_origin_) / dt_;
    if (i_start_ == 0)
        _Dx = buffer_Dx_[i_end_]; // Trivial delta
    else
    {
        Dx_start_ = buffer_Dx_[i_start_];
        Dx_end_ = buffer_Dx_[i_end_];
        minus(Dx_end_, Dx_start_, _Dx);
    }
}

void ProcessorMotion::pushBack(TimeStamp& _ts, Eigen::VectorXs& _dx, Eigen::VectorXs& _Dx_integral)
{
    // Append to buffers
    buffer_ts_.push_back(_ts);
    buffer_dx_.push_back(_dx);
    buffer_Dx_.push_back(_Dx_integral);
}

void ProcessorMotion::eraseFront(TimeStamp& _ts)
{
    i_end_ = (_ts - ts_origin_) / dt_;
    buffer_ts_.erase(buffer_ts_.begin(), buffer_ts_.begin() + i_end_ - 1);
    buffer_dx_.erase(buffer_dx_.begin(), buffer_dx_.begin() + i_end_ - 1);
    buffer_Dx_.erase(buffer_Dx_.begin(), buffer_Dx_.begin() + i_end_ - 1);
}
