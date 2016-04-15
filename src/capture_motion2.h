/**
 * \file capture_motion2.h
 *
 *  Created on: Mar 16, 2016
 *      \author: jsola
 */

#ifndef SRC_CAPTURE_MOTION2_H_
#define SRC_CAPTURE_MOTION2_H_

// Wolf includes
#include "capture_base.h"
#include "motion_buffer.h"

// STL includes
#include <list>
#include <algorithm>
#include <iterator>

namespace wolf {

/** \brief Base class for motion Captures.
 *
 * This class implements Captures for sensors integrating motion.
 *
 * The raw data of this type of captures is required to be in the form of a vector --> see attribute data_.
 *
 * It contains a MotionBuffer buffer of Motion pre-integrated motions that is being filled
 * by the motion processors (deriving from ProcessorMotion) --> See Motion, MotionBuffer, and ProcessorMotion.
 *
 * This buffer contains the integrated motion:
 *  - since the last key-Frame
 *  - until the frame of this capture.
 *
 * Once a keyframe is generated, this buffer is frozen and kept in the Capture for eventual later uses.
 * It is then used to compute the factor that links the Frame of this capture to the previous key-frame in the Trajectory.
 */

class CaptureMotion2 : public CaptureBase
{
        // member types: Motion and MotionBuffer
    public:

        // public interface:
    public:
        CaptureMotion2(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data) :
                CaptureBase(_ts, _sensor_ptr), data_(_data), buffer_()
        {
            //
        }
        virtual ~CaptureMotion2()
        {
            //
        }
        const Eigen::VectorXs& getData() const
        {
            return data_;
        }
        MotionBuffer* getBufferPtr()
        {
            return &buffer_;
        }
        const MotionBuffer* getBufferPtr() const
        {
            return &buffer_;
        }
        const Eigen::VectorXs& getDelta() const{
            return buffer_.getDelta();
        }

        // member data:
    protected:
        Eigen::VectorXs data_; ///< Motion data in form of vector mandatory
        MotionBuffer buffer_; ///< Buffer of motions between this Capture and the next one.
};

} // namespace wolf

#endif /* SRC_CAPTURE_MOTION2_H_ */
