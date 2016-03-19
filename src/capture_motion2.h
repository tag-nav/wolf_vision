/**
 * \file capture_motion2.h
 *
 *  Created on: Mar 16, 2016
 *      \author: jsola
 */

#ifndef SRC_CAPTURE_MOTION2_H_
#define SRC_CAPTURE_MOTION2_H_

#include "capture_base.h"

#include <deque>



/** \brief Base class for motion Captures.
 *
 * This class implements Captures for sensors integrating motion.
 *
 * The raw data of this type of captures is required to be in the form of a vector --> see attribute data_.
 *
 * It contains a buffer of pre-integrated motions that is being filled
 * by the motion processors (deriving from ProcessorMotion).
 *
 * This buffer contains the integrated motion:
 *  - since the last key-Frame
 *  - until the frame of this capture.
 *
 * Once a keyframe is generated and this Capture becomes part of the Wolf tree being solved,
 * this buffer is freezed.
 * It is then used to compute the factor that links the Frame of this capture to the previous key-frame in the Trajectory.
 */
class CaptureMotion2 : public CaptureBase
{
    public:
        typedef struct
        {
            public:
                TimeStamp ts_;       ///< Time stamp
                Eigen::VectorXs Dx_; ///< the integrated delta
        } Motion; ///< One instance of the buffered data, corresponding to a particular time stamp.

        class MotionBuffer{
            public:
                MotionBuffer(const WolfScalar _dt) : dt_(_dt), dt_inv_(1.0/_dt) { }
                void setDt(const WolfScalar _dt) { dt_ = _dt; dt_inv_ = 1/_dt; }
                WolfScalar getDt() const { return dt_; }
                void pushBack(const Motion _motion){container_.push_back(_motion);}
                void pushBack(const TimeStamp _ts, const Eigen::VectorXs& _delta) { container_.push_back(Motion( {_ts, _delta})); }
                void clear(){container_.clear();}
                unsigned int size() const {return container_.size();}
                bool empty() const {return container_.empty();}
                Eigen::VectorXs& getDelta() { return container_.back().Dx_; }
                const Eigen::VectorXs& getDelta() const { return container_.back().Dx_; }
                Eigen::VectorXs& getDelta(const TimeStamp& _ts) { return container_.at(idx(_ts)).Dx_; }
                const Eigen::VectorXs& getDelta(const TimeStamp& _ts) const { return container_.at(idx(_ts)).Dx_; }
            private:
                unsigned int idx(const TimeStamp& _ts) const {return std::lround((_ts - container_.front().ts_) * dt_inv_); } // we rounded to the nearest entry in the buffer
                WolfScalar dt_, dt_inv_;
                std::deque<Motion> container_;
        };

    public:
        CaptureMotion2(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::Vector6s& _data);
        virtual ~CaptureMotion2();

        const Eigen::VectorXs& getData() const;
        MotionBuffer* getBufferPtr();
        const Eigen::VectorXs& getDelta() const;

    protected:
        Eigen::VectorXs data_; ///< Motion data in form of vector mandatory
        MotionBuffer buffer_; ///< Buffer of motions between this Capture and the next one.
};

inline CaptureMotion2::CaptureMotion2(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::Vector6s& _data) :
        CaptureBase(_ts, _sensor_ptr), data_(_data), buffer_(1.0)
{
    //
}

inline CaptureMotion2::~CaptureMotion2()
{
    //
}

inline const Eigen::VectorXs& CaptureMotion2::getData() const
{
    return data_;
}

inline CaptureMotion2::MotionBuffer* CaptureMotion2::getBufferPtr()
{
    return &buffer_;
}

inline const Eigen::VectorXs& CaptureMotion2::getDelta() const
{
    return buffer_.getDelta();
}

#endif /* SRC_CAPTURE_MOTION2_H_ */
