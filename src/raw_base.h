/*
 * RawBase.h
 *
 *  Created on: May 15, 2014
 *      \author: jsola
 */

#ifndef RAW_BASE_H_
#define RAW_BASE_H_

//wolf includes
#include "wolf.h"
#include "time_stamp.h"


/** \brief Base class for all kind of classes storing raw data.
 * 
 * This class is the base for all classes storing raw data.
 * 
 */
class RawBase
{
    private:
        TimeStamp time_stamp_; ///< Time stamp        
        Eigen::VectorXs data_; ///< raw data
        
    protected:
        /** \brief Constructor
         * 
         * Constructor from time stamp
         * \param _ts time stamp
         * 
         */
        RawBase(const WolfScalar _ts);

        /** \brief Constructor
         * 
         * Constructor from time stamp and data
         * \param _ts time stamp
         * \param _data the raw data
         * 
         */
        RawBase(const WolfScalar _ts, const Eigen::VectorXs& _data);

        /** \brief Destructor
         * 
         * Destructor
         * 
         */
        virtual ~RawBase();
        
    public:
        
        /** \brief Gets time stamp
         * 
         * Returns the time stamp
         * 
         **/
        WolfScalar timeStamp() const;

        /** \brief Gets the raw data
         * 
         * Returns a reference to the raw data
         * 
         **/
        const Eigen::VectorXs& rawData() const;
        
        /** \brief Sets raw data
         * 
         * Sets raw data
         * 
         **/
        void setData(unsigned int _size, const WolfScalar *_data);
        
};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////

inline RawBase::RawBase(const WolfScalar _ts) :
        time_stamp_(_ts), 
        data_()
{
    //
}

inline RawBase::RawBase(const WolfScalar _ts, const Eigen::VectorXs& _data) :
        time_stamp_(_ts),
        data_(_data)
{
    //
}

inline RawBase::~RawBase()
{
    //
}

inline WolfScalar RawBase::timeStamp() const
{
    return time_stamp_.get();
}

inline const Eigen::VectorXs& RawBase::rawData() const
{
    return data_;
}

inline void RawBase::setData(unsigned int _size, const WolfScalar *_data)
{
    data_.resize(_size);
    for (unsigned int ii=0; ii<_size; ii++) data_(ii) = *(&_data[ii]);
}

#endif /* RAW_BASE_H_ */
