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
        typedef struct {
            public:
                TimeStamp ts_;       ///< Time stamp
                Eigen::VectorXs Dx_; ///< the integrated delta
        } Motion; ///< One instance of the buffered data, corresponding to a particular time stamp.
        typedef std::deque<Motion> Buffer;

    public:
        CaptureMotion2();
        virtual ~CaptureMotion2();

        const Eigen::VectorXs& getData() const;
        Buffer* getBufferPtr();
        const Eigen::VectorXs& getDelta() const;

    protected:
        Eigen::VectorXs data_; ///< Motion data in form of vector mandatory
        Buffer buffer_; ///< Buffer of motions between this Capture and the next one.
};

inline const Eigen::VectorXs& CaptureMotion2::getData() const
{
    return data_;
}

inline CaptureMotion2::Buffer* CaptureMotion2::getBufferPtr()
{
    return &buffer_;
}

inline const Eigen::VectorXs& CaptureMotion2::getDelta() const
{
    return buffer_.back().Dx_;
}

#endif /* SRC_CAPTURE_MOTION2_H_ */
