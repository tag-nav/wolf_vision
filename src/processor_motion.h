/*
 * \file processor_motion.h
 *
 *  Created on: Dec 17, 2015
 *      author: jsola
 */

#ifndef SRC_PROCESSOR_MOTION_H_
#define SRC_PROCESSOR_MOTION_H_

// Wolf
#include "processor_base.h"
#include "time_stamp.h"
#include "wolf.h"

// STL
#include <deque>


class ProcessorMotion : public ProcessorBase{
    public:
        ProcessorMotion(ProcessorType _tp);
        virtual ~ProcessorMotion();


        virtual void process(CaptureBase* _capture_ptr)
        {
            extractData(_capture_ptr, ts_, data_);
            integrate(data_, dt_);
            pushBack(ts_, dx_, Dx_integral_);
        }

        // Main operations
        void composeDeltaState(const TimeStamp& _t_start, TimeStamp& _t_end, Eigen::VectorXs&);
        void composeState(const TimeStamp& _time_stamp, Eigen::VectorXs&);
        void init(CaptureBase* _origin_ptr);
        void reset(TimeStamp& _ts);

    protected:
        // Helper functions
        void integrate(Eigen::VectorXs& _data, WolfScalar _dt);
        virtual void extractData(CaptureBase* _capture_ptr, TimeStamp& _ts, Eigen::VectorXs& _data) = 0;
        virtual void data2dx(const Eigen::VectorXs& _data, WolfScalar _dt, Eigen::VectorXs& _dx);
        void pushBack(TimeStamp& _ts, Eigen::VectorXs& _dx, Eigen::VectorXs& _Dx_integral);
        void eraseFront(TimeStamp& _ts);
        virtual void plus(const Eigen::VectorXs& _x, const Eigen::VectorXs& _dx, Eigen::VectorXs& _x_plus_dx) = 0;
        virtual void minus(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x0, Eigen::VectorXs& _x1_minus_x0) = 0;

        WolfScalar computeAverageDt();
        WolfScalar getDt();

    protected:
        CaptureBase* origin_ptr_;
        std::deque<TimeStamp> buffer_ts_;
        std::deque<Eigen::VectorXs> buffer_dx_;
        std::deque<Eigen::VectorXs> buffer_Dx_;

    protected:
        // Helper variables: use them to avoid creating temporaries
        TimeStamp ts_; // Time stamp of the data being processed
        TimeStamp ts_origin_; // Time stamp at the origin of buffers
        Eigen::VectorXs data_; // Last received data
        Eigen::VectorXs dx_; // A dx value directly resulting from data
        Eigen::VectorXs Dx_integral_; // The integrated dx's between distant time stamps
        Eigen::VectorXs x_origin_; // The origin state
        Eigen::VectorXs x_other_; // Another state, e.g. x_other_ = plus(x_origin_, Dx_)
        WolfScalar dt_;
        WolfScalar Dt_start_, Dt_end_;
        unsigned int i_start_, i_end_;
        Eigen::VectorXs Dx_start_, Dx_end_;

};

inline void ProcessorMotion::init(CaptureBase* _origin_ptr)
{
    origin_ptr_=_origin_ptr;
    ts_origin_ = _origin_ptr->getTimeStamp();
    x_origin_ = _origin_ptr->getFramePtr()->getState();
}

inline void ProcessorMotion::reset(TimeStamp& _ts)
{
    // Pop data from the front of the buffers before _ts
    // TODO: see if we want to save the popped data somewhere else
    eraseFront(_ts);

    // Reset origin: time-stamp, state and delta_integral
    ts_origin_ = _ts;
    composeState(_ts, x_other_);
    x_origin_ = x_other_;
    Dx_integral_.setZero();
}

inline void ProcessorMotion::integrate(Eigen::VectorXs& _data, WolfScalar _dt)
{
    // Make appropriate delta value from data
    data2dx(_data, _dt, dx_);
    // integrate on top of Dx_
    plus(buffer_Dx_.back(), dx_, Dx_integral_);
}

inline void ProcessorMotion::composeState(const TimeStamp& _time_stamp, Eigen::VectorXs& _x_ts)
{
    // Get time index
    i_end_ = (_time_stamp - ts_origin_) / dt_;

    Dx_end_   = buffer_Dx_[i_end_];
    plus(x_origin_, Dx_end_, _x_ts);
}

/** \brief Convert data to increment vector
 *
 * Overload this method for non-trivial conversions
 */
inline void ProcessorMotion::data2dx(const Eigen::VectorXs& _data, WolfScalar _dt, Eigen::VectorXs& _dx)
{
    assert(_data.size() == _dx.size() && "Sizes do not match");

    // Implement the trivial identity converter.
    _dx = _data * _dt;
}

inline WolfScalar ProcessorMotion::computeAverageDt()
{
    if (buffer_ts_.size() > 0)
        dt_ = (buffer_ts_.back() - buffer_ts_.front()) / (buffer_ts_.size() - 1);
    return dt_;
}

inline WolfScalar ProcessorMotion::getDt()
{
    return dt_;
}

#endif /* SRC_PROCESSOR_MOTION_H_ */
