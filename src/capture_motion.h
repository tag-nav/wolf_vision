/**
 * \file capture_motion2.h
 *
 *  Created on: Mar 16, 2016
 *      \author: jsola
 */

#ifndef SRC_CAPTURE_MOTION_H_
#define SRC_CAPTURE_MOTION_H_

// Wolf includes
#include "capture_base.h"
#include "motion_buffer.h"

// STL includes
#include <list>
#include <algorithm>
#include <iterator>
#include <utility>

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

class CaptureMotion : public CaptureBase
{

        // public interface:

    public:
        CaptureMotion(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data,
                      FrameBase* _origin_frame_ptr = nullptr);

        CaptureMotion(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data,
                      const Eigen::MatrixXs& _data_cov, FrameBase* _origin_frame_ptr = nullptr);

        virtual ~CaptureMotion();

        const Eigen::VectorXs& getData() const;
        const Eigen::MatrixXs& getDataCovariance() const;
        void setData(const Eigen::VectorXs& _data);
        void setDataCovariance(const Eigen::MatrixXs& _data_cov);

        MotionBuffer* getBufferPtr();
        const MotionBuffer* getBufferPtr() const;
        const Eigen::VectorXs& getDelta() const;

        FrameBase* getOriginFramePtr();
        void setOriginFramePtr(FrameBase* _frame_ptr);

        // member data:
    private:
        Eigen::VectorXs data_;        ///< Motion data in form of vector mandatory
        Eigen::MatrixXs data_cov_;    ///< Motion data in form of vector mandatory
        MotionBuffer buffer_;         ///< Buffer of motions between this Capture and the next one.
        FrameBase* origin_frame_ptr_; ///< Pointer to the origin frame of the motion
};

inline CaptureMotion::CaptureMotion(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data,
                                    FrameBase* _origin_frame_ptr) :
        CaptureBase("MOTION", _ts, _sensor_ptr),
        data_(_data),
        data_cov_(Eigen::MatrixXs::Identity(_data.rows(), _data.rows())),
        buffer_(),
        origin_frame_ptr_(_origin_frame_ptr)
{
    //
}

inline CaptureMotion::CaptureMotion(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data,
                                    const Eigen::MatrixXs& _data_cov, FrameBase* _origin_frame_ptr) :
        CaptureBase("MOTION", _ts, _sensor_ptr),
        data_(_data),
        data_cov_(_data_cov),
        buffer_(),
        origin_frame_ptr_(_origin_frame_ptr)
{
    //
}

inline CaptureMotion::~CaptureMotion()
{
    //
}

inline const Eigen::VectorXs& CaptureMotion::getData() const
{
    return data_;
}

inline const Eigen::MatrixXs& CaptureMotion::getDataCovariance() const
{
    return data_cov_;
}

inline void CaptureMotion::setData(const Eigen::VectorXs& _data)
{
    data_ = _data;
}

inline void CaptureMotion::setDataCovariance(const Eigen::MatrixXs& _data_cov)
{
    data_cov_ = _data_cov;
}

inline const wolf::MotionBuffer* CaptureMotion::getBufferPtr() const
{
    return &buffer_;
}

inline wolf::MotionBuffer* CaptureMotion::getBufferPtr()
{
    return &buffer_;
}

inline const Eigen::VectorXs& CaptureMotion::getDelta() const
{
    return buffer_.get().back().delta_integr_;
}

inline wolf::FrameBase* CaptureMotion::getOriginFramePtr()
{
    return origin_frame_ptr_;
}

inline void CaptureMotion::setOriginFramePtr(FrameBase* _frame_ptr)
{
    origin_frame_ptr_ = _frame_ptr;
}

} // namespace wolf

#endif /* SRC_CAPTURE_MOTION_H_ */
