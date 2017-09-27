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
    
WOLF_PTR_TYPEDEFS(CaptureMotion);
    

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
        CaptureMotion(const TimeStamp& _ts, SensorBasePtr _sensor_ptr,
                      const Eigen::VectorXs& _data,
                      Size _delta_size, Size _delta_cov_size, Size _calib_size,
                      FrameBasePtr _origin_frame_ptr = nullptr);

        CaptureMotion(const TimeStamp& _ts, SensorBasePtr _sensor_ptr,
                      const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov,
                      Size _delta_size, Size _delta_cov_size, Size _calib_size,
                      FrameBasePtr _origin_frame_ptr = nullptr);

        virtual ~CaptureMotion();

        // Data
        const Eigen::VectorXs& getData() const;
        const Eigen::MatrixXs& getDataCovariance() const;
        void setData(const Eigen::VectorXs& _data);
        void setDataCovariance(const Eigen::MatrixXs& _data_cov);

        // Buffer
        MotionBuffer& getBuffer();
        const MotionBuffer& getBuffer() const;

        // Buffer's initial conditions for pre-integration
        VectorXs getCalibrationPreint() const;
        VectorXs getCalibration() const;

        FrameBasePtr getOriginFramePtr();
        void setOriginFramePtr(FrameBasePtr _frame_ptr);

        // member data:
    private:
        Eigen::VectorXs data_;          ///< Motion data in form of vector mandatory
        Eigen::MatrixXs data_cov_;      ///< Motion data covariance
        Size calib_size_;
        MotionBuffer buffer_;           ///< Buffer of motions between this Capture and the next one.
        FrameBasePtr origin_frame_ptr_; ///< Pointer to the origin frame of the motion
};

inline CaptureMotion::CaptureMotion(const TimeStamp& _ts,
                                    SensorBasePtr _sensor_ptr,
                                    const Eigen::VectorXs& _data,
                                    Size _delta_size, Size _delta_cov_size, Size _calib_size,
                                    FrameBasePtr _origin_frame_ptr) :
        CaptureBase("MOTION", _ts, _sensor_ptr),
        data_(_data),
        data_cov_(_sensor_ptr ? _sensor_ptr->getNoiseCov() : Eigen::MatrixXs::Zero(_data.rows(), _data.rows())), // Someone should test this zero and do something smart accordingly
        calib_size_(_calib_size),
        buffer_(_data.size(), _delta_size, _delta_cov_size, _calib_size),
        origin_frame_ptr_(_origin_frame_ptr)
{
    //
}

inline CaptureMotion::CaptureMotion(const TimeStamp& _ts,
                                    SensorBasePtr _sensor_ptr,
                                    const Eigen::VectorXs& _data,
                                    const Eigen::MatrixXs& _data_cov,
                                    Size _delta_size, Size _delta_cov_size, Size _calib_size,
                                    FrameBasePtr _origin_frame_ptr) :
        CaptureBase("MOTION", _ts, _sensor_ptr),
        data_(_data),
        data_cov_(_data_cov),
        calib_size_(_calib_size),
        buffer_(_data.size(), _delta_size,_delta_cov_size, _calib_size),
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
    assert(_data.size() == data_.size() && "Wrong size of data vector!");
    data_ = _data;
}

inline void CaptureMotion::setDataCovariance(const Eigen::MatrixXs& _data_cov)
{
    assert(_data_cov.rows() == data_cov_.rows() && "Wrong number of rows of data vector!");
    assert(_data_cov.cols() == data_cov_.cols() && "Wrong number of cols of data vector!");
    data_cov_ = _data_cov;
}

inline const wolf::MotionBuffer& CaptureMotion::getBuffer() const
{
    return buffer_;
}

inline wolf::MotionBuffer& CaptureMotion::getBuffer()
{
    return buffer_;
}

inline wolf::FrameBasePtr CaptureMotion::getOriginFramePtr()
{
    return origin_frame_ptr_;
}

inline void CaptureMotion::setOriginFramePtr(FrameBasePtr _frame_ptr)
{
    origin_frame_ptr_ = _frame_ptr;
}

inline VectorXs CaptureMotion::getCalibrationPreint() const
{
    return getBuffer().getCalibrationPreint();
}

inline VectorXs CaptureMotion::getCalibration() const
{
    VectorXs calib(calib_size_);
    Size index = 0;
    for (Size i = 0; i < getStateBlockVec().size(); i++)
    {
        auto sb = getStateBlockPtr(i);
        if (sb && !sb->isFixed())
        {
            calib.segment(index, sb->getSize()) = sb->getState();
            index += sb->getSize();
        }
    }
    return calib;
}


} // namespace wolf

#endif /* SRC_CAPTURE_MOTION_H_ */
