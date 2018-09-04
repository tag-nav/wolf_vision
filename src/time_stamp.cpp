
#include "time_stamp.h"

namespace wolf {

std::ostream& operator<<(std::ostream& os, const TimeStamp& _ts)
{
    //    std::ios_base::fmtflags fmtfl;
    //
    //    //get/set ostream flags and precision digits
    //    fmtfl = os.flags(std::ios::left);
    //    os.setf(std::ios::fixed, std::ios::floatfield);

    //    std::streamsize nn;
    //    nn = os.precision(TimeStamp::TIME_STAMP_DIGITS_);

    os << _ts.getSeconds() << "." << std::setfill('0') << std::setw(9) << std::right <<_ts.getNanoSeconds(); // write obj to stream
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
        //time_stamp_(_ts)
        time_stamp_nano_(_ts > 0 ? (unsigned long int)(_ts*1e9) : 0)
{
    //
}

TimeStamp::TimeStamp(const unsigned long int& _sec, const unsigned long int& _nsec) :
        //time_stamp_((Scalar)_sec + (Scalar)_nsec/(Scalar)1e9)
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

void TimeStamp::print(std::ostream & ost) const
{
    //std::streamsize nn;
    //std::ios_base::fmtflags fmtfl;
    //
    ////get/set ostream flags and precision digits
    //fmtfl = ost.flags(std::ios::left);
    //ost.setf(std::ios::fixed, std::ios::floatfield);
    //nn = ost.precision(TIME_STAMP_DIGITS_);
    //
    ////send to ostream
    //ost << this->time_stamp_;
    //
    ////restore flags and precision
    //ost.flags(fmtfl);
    //ost.precision(nn);

    ost << *this;
}

} // namespace wolf
