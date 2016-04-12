
#ifndef TIME_STAMP_H_
#define TIME_STAMP_H_

//wolf includes
#include "wolf.h"

#include <sys/time.h>


//C, std
#include <iostream>


namespace wolf {

/**
 * \brief TimeStamp implements basic functionalities for time stamps
 * 
 * TimeStamp implements basic functionalities for time stamps
 */
class TimeStamp
{
    private:
        WolfScalar time_stamp_; ///< Time stamp. Expressed in seconds from 1th jan 1970.
        static const unsigned int TIME_STAMP_DIGITS_ = 10; ///< Number of digits to print time stamp values        

    public:
        /** \brief Constructor
         *
         * Constructor without arguments. Sets time stamp to now.
         *
         */
        TimeStamp();

        /** \brief Constructor with argument
         *
         * Constructor with arguments
         *
         */
        TimeStamp(const WolfScalar _ts);

        /** \brief Constructor from sec and nsec
         *
         * Constructor from sec and nsec
         *
         */
        TimeStamp(const unsigned long int _sec, const unsigned long int _nsec);

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
        void set(const unsigned long int sec, const unsigned long int nanosec);

        /** \brief Set time stamp
         *
         * Sets time stamp to a given value passed as a scalar_t (seconds)
         *
         */
        void set(const WolfScalar ts);

        /** \brief Get time stamp
         *
         * Returns time stamp
         *
         */
        WolfScalar get() const;

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
        void operator =(const WolfScalar& ts);

        /** \brief Assignement operator
         * 
         * Assignement operator
         * 
         */
        void operator =(const TimeStamp& ts);

        /** \brief comparison operator
         * 
         * Comparison operator
         * 
         */
        bool operator <(const TimeStamp& ts) const;

        /** \brief comparison operator
         * 
         * Comparison operator
         * 
         */
        bool operator <=(const TimeStamp& ts) const;

        /** \brief difference operator
         * 
         * difference operator
         * 
         */
        WolfScalar operator -(const TimeStamp& ts) const;

        /** \brief Add-assign operator
         */
        void operator +=(const WolfScalar& dt);

        /** \brief Add-assign operator
         */
        TimeStamp operator +(const WolfScalar& dt);

        /** \brief Prints time stamp to a given ostream
         *
         * Prints time stamp to a given ostream
         *
         */
        void print(std::ostream & ost = std::cout) const;

};

inline void TimeStamp::setToNow()
{
    timeval ts;
    gettimeofday(&ts, NULL);
    time_stamp_ = (WolfScalar)(ts.tv_sec) + (WolfScalar)(ts.tv_usec) / 1e6;
}

inline void TimeStamp::set(const WolfScalar ts)
{
    time_stamp_ = ts;
}

inline void TimeStamp::set(const unsigned long int sec, const unsigned long int nanosec)
{
    time_stamp_ = (WolfScalar)(sec) + (WolfScalar)(nanosec) / (WolfScalar)(1e9);
}

inline void TimeStamp::set(const timeval& ts)
{
    time_stamp_ = (WolfScalar)(ts.tv_sec) + (WolfScalar)(ts.tv_usec) / 1e6;
}

inline WolfScalar TimeStamp::get() const
{
    return time_stamp_;
}

inline unsigned long int TimeStamp::getSeconds() const
{
    unsigned long int ts;
    ts = (unsigned long int)((floor(time_stamp_)));
    return ts;
}

inline unsigned long int TimeStamp::getNanoSeconds() const
{
    WolfScalar ts;
    ts = (WolfScalar)((floor(time_stamp_)));
    return (unsigned long int)((time_stamp_ - ts) * 1e9);
}

inline void TimeStamp::operator =(const TimeStamp& ts)
{
    time_stamp_ = ts.get();
}

inline void TimeStamp::operator =(const WolfScalar& ts)
{
    time_stamp_ = ts;
}

inline bool TimeStamp::operator <(const TimeStamp& ts) const
{
    if (time_stamp_ < ts.get())
        return true;
    else
        return false;
}

inline bool TimeStamp::operator <=(const TimeStamp& ts) const
{
    if (time_stamp_ <= ts.get())
        return true;
    else
        return false;
}

inline WolfScalar TimeStamp::operator -(const TimeStamp& ts) const
{
    return (time_stamp_ - ts.get());
}

inline void TimeStamp::operator +=(const WolfScalar& dt)
{
    time_stamp_ += dt;
}

inline TimeStamp TimeStamp::operator +(const WolfScalar& dt)
{
    return TimeStamp(time_stamp_ + dt);
}

} // namespace wolf

#endif
