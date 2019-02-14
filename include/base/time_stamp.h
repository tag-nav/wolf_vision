
#ifndef TIME_STAMP_H_
#define TIME_STAMP_H_

//wolf includes
#include "base/wolf.h"

//C, std
#include <sys/time.h>
#include <iostream>
#include <iomanip>

static const unsigned int NANOSECS = 1000000000;

namespace wolf {

/**
 * \brief TimeStamp implements basic functionalities for time stamps
 * 
 * TimeStamp implements basic functionalities for time stamps
 */
class TimeStamp
{
    protected:
        unsigned long int time_stamp_nano_; ///< Time stamp. Expressed in nanoseconds from 1th jan 1970.

    public:
        /** \brief Constructor
         *
         * Constructor without arguments. Sets time stamp to now.
         *
         */
        TimeStamp();

        /** \brief Copy constructor
         *
         * Copy constructor
         *
         */
        TimeStamp(const TimeStamp& _ts);

        /** \brief Constructor with argument
         *
         * Constructor with arguments
         *
         */
        TimeStamp(const Scalar& _ts);

        /** \brief Constructor from sec and nsec
         *
         * Constructor from sec and nsec
         *
         */
        TimeStamp(const unsigned long int& _sec, const unsigned long int& _nsec);

        /** \brief Destructor
         *
         * Destructor
         *
         */
        ~TimeStamp();

        /** \brief Time stamp to now
         */
        void setToNow();

        /** \brief Set time stamp
         *
         * Sets time stamp to a given value passed as a timeval struct
         */
        void set(const timeval& ts);

        /** \brief Set time stamp
         *
         * Sets time stamp to a given value passed as a two-integer (seconds and nanoseconds)
         *
         */
        void set(const unsigned long int& sec, const unsigned long int& nanosec);

        /** \brief Set time stamp
         *
         * Sets time stamp to a given value passed as a scalar_t (seconds)
         *
         */
        void set(const Scalar& ts);

        /** \brief Get time stamp
         *
         * Returns time stamp
         *
         */
        Scalar get() const;

        /** \brief Get time stamp (only seconds)
         *
         * Returns seconds of time stamp
         *
         */
        unsigned long int getSeconds() const;

        /** \brief Get time stamp (only nano seconds)
         *
         * Returns nanoseconds part of time stamp
         *
         */
        unsigned long int getNanoSeconds() const;

        /** \brief Assignement operator
         * 
         * Assignement operator
         * 
         */
        void operator =(const Scalar& ts);

        /** \brief Assignement operator
         * 
         * Assignement operator given a scalar_t (seconds)
         * 
         */
        void operator =(const TimeStamp& ts);

        /** \brief comparison operator
         * 
         * Comparison operator
         * 
         */
        bool operator !=(const TimeStamp& ts) const;
        bool operator ==(const TimeStamp& ts) const;
        bool operator <(const TimeStamp& ts) const;
        bool operator >(const TimeStamp& ts) const;

        /** \brief comparison operator
         * 
         * Comparison operator
         * 
         */
        bool operator <=(const TimeStamp& ts) const;
        bool operator >=(const TimeStamp& ts) const;

        /** \brief Add-assign operator given a scalar_t (seconds)
         */
        void operator +=(const Scalar& dt);

        /** \brief Add-assign operator given a scalar_t (seconds)
         */
        TimeStamp operator +(const Scalar& dt) const;

        /** \brief Substraction-assign operator given a scalar_t (seconds)
         */
        void operator -=(const Scalar& dt);

        /** \brief difference operator
         * 
         * difference operator that returns a scalar_t (seconds)
         * 
         */
        TimeStamp operator -(const Scalar& ts) const;

        /** \brief difference operator
         *
         * difference operator that returns a Timestamp (seconds)
         *
         */
        Scalar operator -(const TimeStamp& ts) const;

        /** \brief Prints time stamp to a given ostream
         *
         * Prints time stamp to a given ostream
         *
         */
        void print(std::ostream & ost = std::cout) const;

        friend std::ostream& operator<<(std::ostream& os, const TimeStamp& _ts);

};

inline void TimeStamp::set(const Scalar& ts)
{
    time_stamp_nano_ = (ts > 0 ? (unsigned long int)(ts*NANOSECS) : 0);
}

inline void TimeStamp::set(const unsigned long int& sec, const unsigned long int& nanosec)
{
    time_stamp_nano_ = sec*NANOSECS+nanosec;
}

inline void TimeStamp::set(const timeval& ts)
{
    time_stamp_nano_ = (unsigned long int)(ts.tv_sec*NANOSECS) + (unsigned long int)(ts.tv_usec*1000);
}

inline Scalar TimeStamp::get() const
{
    return ((Scalar)( time_stamp_nano_))*1e-9;
}

inline unsigned long int TimeStamp::getSeconds() const
{
    return time_stamp_nano_ / NANOSECS;
}

inline unsigned long int TimeStamp::getNanoSeconds() const
{
    return time_stamp_nano_ % NANOSECS;
}

inline void TimeStamp::operator =(const TimeStamp& ts)
{
    time_stamp_nano_ = ts.time_stamp_nano_;
}

inline void TimeStamp::operator =(const Scalar& ts)
{
    time_stamp_nano_ = (unsigned long int)(ts*NANOSECS);
}

inline bool TimeStamp::operator ==(const TimeStamp& ts) const
{
    return (time_stamp_nano_ == ts.time_stamp_nano_);
}

inline bool TimeStamp::operator !=(const TimeStamp& ts) const
{
    return (time_stamp_nano_ != ts.time_stamp_nano_);
}

inline bool TimeStamp::operator <(const TimeStamp& ts) const
{
    return (time_stamp_nano_ < ts.time_stamp_nano_);
}

inline bool TimeStamp::operator >(const TimeStamp& ts) const
{
    return (time_stamp_nano_ > ts.time_stamp_nano_);
}

inline bool TimeStamp::operator <=(const TimeStamp& ts) const
{
    return (time_stamp_nano_ <= ts.time_stamp_nano_);
}

inline bool TimeStamp::operator >=(const TimeStamp& ts) const
{
    return (time_stamp_nano_ >= ts.time_stamp_nano_);
}

inline void TimeStamp::operator +=(const Scalar& dt)
{
    time_stamp_nano_ += (unsigned long int)(dt*NANOSECS);
}

inline Scalar TimeStamp::operator -(const TimeStamp& ts) const
{
    return Scalar((long int)(time_stamp_nano_ - ts.time_stamp_nano_))*1e-9; // long int cast fix overflow in case of negative substraction result
}

static const TimeStamp InvalidStamp(-1,-1);

} // namespace wolf

#endif
