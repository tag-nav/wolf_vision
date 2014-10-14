/**
 * \file capture_laser_2D.h
 *
 *  Created on: 06/10/2014
 *     \author: acorominas
 */

#ifndef CAPTURE_LASER_2D_H_
#define CAPTURE_LASER_2D_H_

#include "capture.h"

class CaptureLaser2D : public Capture
{
    public:
        /** \brief Constructor
         * 
         * Constructor
         * 
         **/
        CaptureLaser2D(const FrameShPtr& _frm_ptr, const SensorShPtr& _sen_ptr, const NodeLocation _loc = MID);

        /** \brief Destructor
         * 
         * Destructor
         * 
         **/        
        virtual ~CaptureLaser2D();
        
        /** \brief Computes error
         * 
         * Computes error
         * TODO: Check if this is the appropiate interface. There ara 3 more interfaces at NodeConstrainer to be overloaded.
         * 
         **/
        virtual void computeError();
        
};

////////////////////////////////
// IMPLEMENTATION
////////////////////////////////


inline CaptureLaser2D::CaptureLaser2D(const FrameShPtr& _frm_ptr, const SensorShPtr& _sen_ptr, const NodeLocation _loc) :
        Capture(_frm_ptr, _sen_ptr, _loc)
{
    // 
}

inline CaptureLaser2D::~CaptureLaser2D()
{
    //
}

inline virtual void computeError()
{
    //TODO
}

#endif /* CAPTURE_LASER_2D_H_ */
