/*
 * \file processor_motion.h
 *
 *  Created on: Dec 17, 2015
 *      author: jsola
 */

#ifndef SRC_PROCESSOR_MOTION_H_
#define SRC_PROCESSOR_MOTION_H_

#include "time_stamp.h"

class ProcessorMotion : public ProcessorBase{
    public:
        ProcessorMotion();
        virtual ~ProcessorMotion();
        void integrate(TimeStamp& _time_stamp);
        void computeRelativeState(TimeStamp& _t_start, TimeStamp& _t_end);
        void computeAbsoluteState(TimeStamp& _time_stamp);
};
#endif /* SRC_PROCESSOR_MOTION_H_ */
