
#include "time_stamp.h"


TimeStamp::TimeStamp() :
        time_stamp_(0)
{
    setToNow();
}

TimeStamp::TimeStamp(const WolfScalar _ts) :
        time_stamp_(_ts)
{
    //
}

TimeStamp::TimeStamp(const unsigned long int _sec, const unsigned long int _nsec) :
        time_stamp_((WolfScalar)_sec + (WolfScalar)_nsec/(WolfScalar)1e9)
{
    //
}

TimeStamp::~TimeStamp()
{
    //nothing to do
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
