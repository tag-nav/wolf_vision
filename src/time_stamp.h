
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
        Scalar time_stamp_; ///< Time stamp. Expressed in seconds from 1th jan 1970.
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
        TimeStamp(const Scalar _ts);

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
        void set(const Scalar ts);

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
        bool operator <(const TimeStamp& ts) const;

        /** \brief comparison operator
         * 
         * Comparison operator
         * 
         */
        bool operator <=(const TimeStamp& ts) const;

        /** \brief difference operator
         * 
         * difference operator that returns a scalar_t (seconds)
         * 
         */
        Scalar operator -(const TimeStamp& ts) const;

        /** \brief Add-assign operator given a scalar_t (seconds)
         */
        void operator +=(const Scalar& dt);

        /** \brief Add-assign operator given a scalar_t (seconds)
         */
        TimeStamp operator +(const Scalar& dt);

        /** \brief Add-assign operator given a Timestamp
         */
        TimeStamp operator +(const TimeStamp& dt);

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
    time_stamp_ = (Scalar)(ts.tv_sec) + (Scalar)(ts.tv_usec) / 1e6;
}

inline void TimeStamp::set(const Scalar ts)
{
    time_stamp_ = ts;
}

inline void TimeStamp::set(const unsigned long int sec, const unsigned long int nanosec)
{
    time_stamp_ = (Scalar)(sec) + (Scalar)(nanosec) / (Scalar)(1e9);
}

inline void TimeStamp::set(const timeval& ts)
{
    time_stamp_ = (Scalar)(ts.tv_sec) + (Scalar)(ts.tv_usec) / 1e6;
}

inline Scalar TimeStamp::get() const
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
    Scalar ts;
    ts = (Scalar)((floor(time_stamp_)));
    return (unsigned long int)((time_stamp_ - ts) * 1e9);
}

inline void TimeStamp::operator =(const TimeStamp& ts)
{
    time_stamp_ = ts.get();
}

inline void TimeStamp::operator =(const Scalar& ts)
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

inline Scalar TimeStamp::operator -(const TimeStamp& ts) const
{
    return (time_stamp_ - ts.get());
}

inline void TimeStamp::operator +=(const Scalar& dt)
{
    time_stamp_ += dt;
}

inline TimeStamp TimeStamp::operator +(const Scalar& dt)
{
    return TimeStamp(time_stamp_ + dt);
}

} // namespace wolf

#endif
