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

// STL includes
#include <list>
#include <algorithm>
#include <iterator>

/** \brief Base class for motion Captures.
 * \param MotionDeltaType The type of the motion delta and the motion integrated delta. It can be an Eigen::VectorXs (default) or any other construction, most likely a struct.
 *        Generalized Delta types allow for optimized algorithms. For example, in an IMU, the Delta type might be defined as:
 * \code
 *   typedef struct {
 *     Eigen::Vector3s dp;     // Position delta
 *     Eigen::Matrix3s dR;     // Rotation matrix delta
 *     Eigen::Vector3s dv;     // Velocity delta
 *     Eigen::Vector3s dab;    // Acc. bias delta
 *     Eigen::Vector3s dwb;    // Gyro bias delta
 *   } ImuDeltaType;
 * \endcode
 *     Also, using quaternions instead of rotation matrices
 * \code
 *   typedef struct {
 *     Eigen::Vector3s    dp;  // Position delta
 *     Eigen::Quaternions dq;  // Quaternion delta
 *     Eigen::Vector3s    dv;  // Velocity delta
 *     Eigen::Vector3s    dab; // Acc. bias delta
 *     Eigen::Vector3s    dwb; // Gyro bias delta
 *   } ImuDeltaType;
 * \endcode
 *
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
 * Once a keyframe is generated, this buffer is frozen and kept in the Capture for eventual later uses.
 * It is then used to compute the factor that links the Frame of this capture to the previous key-frame in the Trajectory.
 */
template <class MotionDeltaType = Eigen::VectorXs>
class CaptureMotion2 : public CaptureBase
{
    public:
        typedef struct
        {
            public:
                TimeStamp ts_;       ///< Time stamp
                MotionDeltaType Dx_; ///< the integrated delta
        } Motion; ///< One instance of the buffered data, corresponding to a particular time stamp.

        class MotionBuffer{
            public:
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
                 * Once a keyframe is generated, this buffer is frozen and kept in the Capture for eventual later uses.
                 * It is then used to compute the factor that links the Frame of this capture to the previous key-frame in the Trajectory.
                 */
                MotionBuffer(){}
                void pushBack(const Motion _motion){container_.push_back(_motion);}
                void pushBack(const TimeStamp _ts, const MotionDeltaType& _delta) { container_.push_back(Motion( {_ts, _delta})); }
                void clear(){container_.clear();}
                unsigned int size() const {return container_.size();}
                bool empty() const {return container_.empty();}
                const TimeStamp& getTimeStamp() const {return container_.back().ts_;}
                const MotionDeltaType& getDelta() const { return container_.back().Dx_; }
                MotionDeltaType& getDelta(const TimeStamp& _ts) {
                    typename std::list<Motion>::iterator next = std::find_if (container_.begin(), container_.end(), [&](const Motion& m){return _ts<=m.ts_;});
                    if (next == container_.end())
                        next--;
                    return next->Dx_;
                }
                const MotionDeltaType& getDelta(const TimeStamp& _ts) const {
                    typename std::list<Motion>::const_iterator next = std::find_if (container_.begin(), container_.end(), [&](const Motion& m){return _ts<=m.ts_;});
                    if (next == container_.end())
                        next--;
                    return next->Dx_;
                }

            private:
                std::list<Motion> container_;
        };

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
        CaptureMotion2::MotionBuffer* getBufferPtr()
        {
            return &buffer_;
        }
        const CaptureMotion2::MotionBuffer* getBufferPtr() const
        {
            return &buffer_;
        }
        const MotionDeltaType& getDelta() const{
            return buffer_.getDelta();
        }

    protected:
        Eigen::VectorXs data_; ///< Motion data in form of vector mandatory
        MotionBuffer buffer_; ///< Buffer of motions between this Capture and the next one.
};


#endif /* SRC_CAPTURE_MOTION2_H_ */
