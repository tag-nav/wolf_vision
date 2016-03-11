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

        // TODO: These thre fncns will use plus() and minus() below
        void integrate(TimeStamp& _time_stamp){};
        void getDeltaState(TimeStamp& _t_start, TimeStamp& _t_end){};
        void getState(TimeStamp& _time_stamp){};

    protected:
        // TODO: Think about the API of these:
        void plus() = 0;
        void minus() = 0;
};
#endif /* SRC_PROCESSOR_MOTION_H_ */
