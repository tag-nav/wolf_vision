/*
 * \file processor_laser_2D.h
 *
 *  Created on: Dec 15, 2015
 *      author: jsola
 */

#ifndef SRC_PROCESSOR_LASER_2D_H_
#define SRC_PROCESSOR_LASER_2D_H_

#include "processor_base.h"

class ProcessorLaser2D : public ProcessorBase
{
    public:
        ProcessorLaser2D();
        virtual ~ProcessorLaser2D();

        void extractFeatures(CaptureBase* capture_ptr_);
        void establishConstraints(CaptureBase* capture_ptr_);
};
#endif /* SRC_PROCESSOR_LASER_2D_H_ */
