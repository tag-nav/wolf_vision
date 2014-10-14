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
    private:

    public:
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * 
         * 
         **/
        RawLaser2D(const WolfScalar _ts);
        
        /** \brief Destructor
         * 
         * Destructor
         * 
         **/
        virtual ~RawLaser2D();

};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////

inline RawLaser2D::RawLaser2D(const WolfScalar _ts) :
        RawBase(_ts)
{
    //
}

inline RawLaser2D::~RawLaser2D()
{
    //
}

#endif /*RAW_LASER_2D_H_*/
