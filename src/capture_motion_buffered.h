/*
 * \file capture_motion_buffered.h
 *
 *  Created on: Dec 18, 2015
 *      author: jsola
 */

#ifndef SRC_CAPTUREMOTIONBUFFERED_H_
#define SRC_CAPTUREMOTIONBUFFERED_H_

class CaptureMotionBuffered : public CaptureBase
{
    protected:
        Eigen::VectorXs initial_state_;
        Eigen::VectorXs current_state_;
        Eigen::VectorXs delta_state_;
    public:
        CaptureMotionBuffered(){}
        virtual ~CaptureMotionBuffered(){}

        void addCapture(){}
        void pruneBuffer(TimeStamp& _ts){}
        void integrteDeltaState(TimeStamp& _ts, TimeStamp& _ts_init = TimeStamp(0.0)){}
        void resetDeltaState(TimeStamp* _ts){}
        void computeState(TimeStamp& _ts){}
        Eigen::VectorXs& getState(){return current_state_;}
};



#endif /* SRC_CAPTUREMOTIONBUFFERED_H_ */
