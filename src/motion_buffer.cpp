#include "motion_buffer.h"
namespace wolf
{

Motion::Motion():
        ts_(0)
{
    resize(0);
}

Motion::Motion(const TimeStamp& _ts, const VectorXs& _delta, const VectorXs& _delta_int, const MatrixXs& _jac_delta, const MatrixXs& _jac_delta_int, const MatrixXs& _delta_cov) :
        ts_(_ts),
        delta_(_delta),
        delta_integr_(_delta_int),
        jacobian_delta_(_jac_delta),
        jacobian_delta_integr_(_jac_delta_int),
        delta_cov_(_delta_cov)
{
}

Motion::Motion(const TimeStamp& _ts, Size _delta_size, Size _cov_size) :
        ts_(_ts)
{
    resize(_delta_size, _cov_size == 0 ? _delta_size : _cov_size);
}

Motion::~Motion()
{
}

void Motion::resize(Size ds)
{
    resize(ds, ds);
}

void Motion::resize(Size ds, Size dcs)
{
    delta_.resize(ds);
    delta_integr_.resize(ds);
    jacobian_delta_.resize(dcs, dcs);
    jacobian_delta_integr_.resize(dcs, dcs);
    delta_cov_.resize(dcs, dcs);
    delta_integr_cov_.resize(dcs, dcs);
}

const Motion& MotionBuffer::getMotion(const TimeStamp& _ts) const
{
    //assert((container_.front().ts_ <= _ts) && "Query time stamp out of buffer bounds");
    auto previous = std::find_if(container_.rbegin(), container_.rend(), [&](const Motion& m)
    {
        return m.ts_ <= _ts;
    });
    if (previous == container_.rend())
        // The time stamp is older than the buffer's oldest data.
        // We could do something here, and throw an error or something, but by now we'll return the first valid data
        previous--;

    return *previous;
}

void MotionBuffer::getMotion(const TimeStamp& _ts, Motion& _motion) const
{
    //assert((container_.front().ts_ <= _ts) && "Query time stamp out of buffer bounds");
    auto previous = std::find_if(container_.rbegin(), container_.rend(), [&](const Motion& m)
    {
        return m.ts_ <= _ts;
    });
    if (previous == container_.rend())
        // The time stamp is older than the buffer's oldest data.
        // We could do something here, but by now we'll return the last valid data
        previous--;

    _motion = *previous;
}

void MotionBuffer::split(const TimeStamp& _ts, MotionBuffer& _oldest_buffer)
{
    assert((container_.front().ts_ <= _ts) && "Query time stamp out of buffer bounds");
    auto previous = std::find_if(container_.rbegin(), container_.rend(), [&](const Motion& m)
    {
        return m.ts_ <= _ts;
    });
    if (previous == container_.rend())
    {
        // The time stamp is more recent than the buffer's most recent data:
        // return an empty buffer as the _oldest_buffer
        _oldest_buffer.get().clear();
    }
    else
    {
        // Transfer part of the buffer
        _oldest_buffer.container_.splice(_oldest_buffer.container_.begin(), container_, container_.begin(),
                                         (previous--).base());
    }
}

}
