
#include "time_stamp.h"

namespace wolf {

std::ostream& operator<<(std::ostream& os, const TimeStamp& _ts)
{
    // Store cout flags
    std::ios_base::fmtflags cout_flags(std::cout.flags());

    os << _ts.getSeconds() << "." << std::setfill('0') << std::setw(9) << std::right <<_ts.getNanoSeconds(); // write obj to stream

    // Restore initial cout flags
    std::cout.flags(cout_flags);
    return os;
}



TimeStamp::TimeStamp() :
        //time_stamp_(0)
        time_stamp_nano_(0)
{
    setToNow();
}

TimeStamp::TimeStamp(const TimeStamp& _ts) :
        time_stamp_nano_(_ts.time_stamp_nano_)
{
    //
}

TimeStamp::TimeStamp(const Scalar& _ts) :
        time_stamp_nano_(_ts > 0 ? (unsigned long int)(_ts*1e9) : 0)
{
    //
}

TimeStamp::TimeStamp(const unsigned long int& _sec, const unsigned long int& _nsec) :
        time_stamp_nano_(_sec*NANOSECS+_nsec)
{
    //
}

TimeStamp::~TimeStamp()
{
    //nothing to do
}

void TimeStamp::setToNow()
{
    timeval ts;
    gettimeofday(&ts, NULL);
    set(ts);
}

TimeStamp TimeStamp::operator +(const Scalar& dt) const
{
    TimeStamp ts(*this);
    ts += dt;
    return ts;
}

TimeStamp TimeStamp::operator -(const Scalar& dt) const
{
    TimeStamp ts(*this);
    ts -= dt;
    return ts;
}

inline void TimeStamp::operator -=(const Scalar& dt)
{
    unsigned long int dt_nano = (unsigned long int)(dt*NANOSECS);
    time_stamp_nano_ = (dt_nano > time_stamp_nano_ ? 0 : time_stamp_nano_ - dt_nano);
}

void TimeStamp::print(std::ostream & ost) const
{
    ost << *this;
}

} // namespace wolf
