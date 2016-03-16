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

        Buffer* getBufferPtr();
        Eigen::VectorXs& getDelta();

    protected:
        Eigen::VectorXs data_; ///< Motion data in form of vector mandatory
        Buffer buffer_; ///< Buffer of motions between this Capture and the next one.
};

inline CaptureMotion2::Buffer* CaptureMotion2::getBufferPtr()
{
    return &buffer_;
}

inline Eigen::VectorXs& CaptureMotion2::getDelta()
{
    return buffer_.back().Dx_;
}

#endif /* SRC_CAPTURE_MOTION2_H_ */
