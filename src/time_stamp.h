
#ifndef TIME_STAMP_H_
#define TIME_STAMP_H_

//C, std
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <iostream>

//wolf
#include "wolf.h"


/**
 *
 * \brief TimeStamp implements basic functionalities for time stamps
 * 
 * TimeStamp implements basic functionalities for time stamps
 *
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
        virtual ~TimeStamp();

        /** \brief Time stamp to now
         *
         * Sets time stamp to now
         *
         */
        void setToNow();

        /** \brief Set time stamp
         *
         * Sets time stamp to a given value passed as a timeval struct
         *
         */
        void set(const timeval & ts);

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

        /** \brief Prints time stamp to a given ostream 
         * 
         * Prints time stamp to a given ostream 
         * 
         */
        void print(std::ostream & ost = std::cout) const;

        /** \brief Assignement operator
         * 
         * Assignement operator
         * 
         */
        void operator=(const WolfScalar & ts);

        /** \brief Assignement operator
         * 
         * Assignement operator
         * 
         */
        void operator=(const TimeStamp & ts);

        /** \brief comparison operator
         * 
         * Comparison operator
         * 
         */
        bool operator<(const TimeStamp & ts) const;

        /** \brief comparison operator
         * 
         * Comparison operator
         * 
         */
        bool operator<=(const TimeStamp & ts) const;

        /** \brief difference operator
         * 
         * difference operator
         * 
         */
        WolfScalar operator-(const TimeStamp & ts) const;

};
#endif
