
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

    os << _ts.time_stamp_; // write obj to stream
    return os;
}



TimeStamp::TimeStamp() :
        time_stamp_(0)
{
    setToNow();
}

TimeStamp::TimeStamp(const Scalar _ts) :
        time_stamp_(_ts)
{
    //
}

TimeStamp::TimeStamp(const unsigned long int _sec, const unsigned long int _nsec) :
        time_stamp_((Scalar)_sec + (Scalar)_nsec/(Scalar)1e9)
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
    time_stamp_ = (Scalar)((ts.tv_sec)) + (Scalar)((ts.tv_usec)) / 1e6;
}

unsigned long int TimeStamp::getSeconds() const
{
    unsigned long int ts;
    ts = (unsigned long int)(((floor(time_stamp_))));
    return ts;
}

unsigned long int TimeStamp::getNanoSeconds() const
{
    Scalar ts;
    ts = (Scalar)(((floor(time_stamp_))));
    return (unsigned long int)(((time_stamp_ - ts) * 1e9));
}

void TimeStamp::print(std::ostream & ost) const
{
    std::streamsize nn;
    std::ios_base::fmtflags fmtfl;

    //get/set ostream flags and precision digits
    fmtfl = ost.flags(std::ios::left);
    ost.setf(std::ios::fixed, std::ios::floatfield);
    nn = ost.precision(TIME_STAMP_DIGITS_);

    //send to ostream
    ost << this->time_stamp_;

    //restore flags and precision
    ost.flags(fmtfl);
    ost.precision(nn);
}

} // namespace wolf
