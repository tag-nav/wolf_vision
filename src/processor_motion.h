/*
 * \file processor_motion.h
 *
 *  Created on: Dec 17, 2015
 *      author: jsola
 */

#ifndef SRC_PROCESSOR_MOTION_H_
#define SRC_PROCESSOR_MOTION_H_

#include "time_stamp.h"
#include "wolf.h"

class ProcessorMotion : public ProcessorBase{
    public:
        ProcessorMotion();
        virtual ~ProcessorMotion();

        // TODO: Implement the pure virtual from base
        virtual void process(CaptureBase* _capture_ptr){};

        // TODO: These three functions will use plus() and minus() below
        void integrate(TimeStamp& _time_stamp){};
        void composeDeltaState(const TimeStamp& _t_start, TimeStamp& _t_end, Eigen::VectorXs&){};
        void composeState(const TimeStamp& _time_stamp, Eigen::VectorXs&){};

    protected:
        // TODO: Think about the API of these:
        virtual void plus(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta) = 0;
        virtual void minus(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x0, Eigen::VectorXs& _x1_minus_x0) = 0;

        void composeOriginState(Eigen::VectorXs&);
};
#endif /* SRC_PROCESSOR_MOTION_H_ */
