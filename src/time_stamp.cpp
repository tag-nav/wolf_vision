/*
 * time_stamp.cpp
 *
 *  Created on: May 26, 2014
 *      \author: andreu
 */

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

TimeStamp::~TimeStamp()
{
    //nothing to do
}

void TimeStamp::setToNow()
{
    timeval ts;
    gettimeofday(&ts, NULL);
    time_stamp_ = (WolfScalar)(ts.tv_sec + ts.tv_usec / 1e6);
}

void TimeStamp::set(const timeval & ts)
{
    time_stamp_ = (WolfScalar)(ts.tv_sec + ts.tv_usec / 1e6);
}

void TimeStamp::set(const unsigned long int sec, const unsigned long int nanosec)
{
    time_stamp_ = (WolfScalar)(sec + nanosec / 1e9);
}

void TimeStamp::set(const WolfScalar ts)
{
    time_stamp_ = ts;
}

WolfScalar TimeStamp::get() const
{
    return time_stamp_;
}

unsigned long int TimeStamp::getSeconds() const
{
    unsigned long int ts;
    ts = (unsigned long int)floor(time_stamp_);
    return ts;
}

unsigned long int TimeStamp::getNanoSeconds() const
{
    WolfScalar ts;
    ts = floor(time_stamp_);
    return (unsigned long int)((time_stamp_ - ts) * 1e9);
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

void TimeStamp::operator=(const WolfScalar ts)
{
    time_stamp_ = ts;
}

void TimeStamp::operator=(const TimeStamp ts)
{
    time_stamp_ = ts.get();
}

bool TimeStamp::operator<(const TimeStamp ts) const
{
    if (time_stamp_ < ts.get())
        return true;
    else
        return false;
}

bool TimeStamp::operator<=(const TimeStamp ts) const
{
    if (time_stamp_ <= ts.get())
        return true;
    else
        return false;
}

WolfScalar TimeStamp::operator-(const TimeStamp ts) const
{
    return (time_stamp_ - ts.get());
}

