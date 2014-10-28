/**
 * \file raw_laser_2D.h
 *
 *  Created on: 06/10/2014
 *     \author: acorominas
 */

#ifndef RAW_LASER_2D_H_
#define RAW_LASER_2D_H_

#include "raw_base.h"

class RawLaser2D : public RawBase
{
    protected:

    public:
        /** \brief Constructor without arguments
         * 
         * Constructor without arguments. Time stamp set to now.
         * 
         **/
        RawLaser2D();
        
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * 
         **/
        RawLaser2D(const WolfScalar _ts);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~RawLaser2D();
        
        /** \brief Prints time stamp + data
         * 
         * Prints "LASER2D" , then calls RawBase::print()
         * 
         **/
        virtual void print(std::ostream & ost = std::cout) const;

};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////

inline RawLaser2D::RawLaser2D() :
        RawBase()
{
    //
}

inline RawLaser2D::RawLaser2D(const WolfScalar _ts) :
        RawBase(_ts)
{
    //
}

inline RawLaser2D::~RawLaser2D()
{
    //
}

inline void RawLaser2D::print(std::ostream & ost) const
{
    ost << "LASER2D: ";
    RawBase::print();
}

#endif /*RAW_LASER_2D_H_*/
