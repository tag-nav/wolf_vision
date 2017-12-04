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
                      Size _delta_size,
                      Size _delta_cov_size,
                      FrameBasePtr _origin_frame_ptr);

        CaptureMotion(const TimeStamp& _ts, SensorBasePtr _sensor_ptr,
                      const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov,
                      Size _delta_size,
                      Size _delta_cov_size,
                      FrameBasePtr _origin_frame_ptr,
                      StateBlockPtr _p_ptr = nullptr,
                      StateBlockPtr _o_ptr = nullptr,
                      StateBlockPtr _intr_ptr = nullptr);

        virtual ~CaptureMotion();

        // Type
        virtual bool isMotion() const override { return true; }

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
        void setCalibrationPreint(const VectorXs& _calib_preint);
        MatrixXs getJacobianCalib();
        MatrixXs getJacobianCalib(const TimeStamp& _ts);

        // Get delta preintegrated, and corrected for changes on calibration params
        VectorXs getDeltaCorrected(const VectorXs& _calib_current);
        VectorXs getDeltaCorrected(const VectorXs& _calib_current, const TimeStamp& _ts);
        VectorXs getDeltaPreint();
        VectorXs getDeltaPreint(const TimeStamp& _ts);
        MatrixXs getDeltaPreintCov();
        MatrixXs getDeltaPreintCov(const TimeStamp& _ts);
        virtual VectorXs correctDelta(const VectorXs& _delta, const VectorXs& _delta_error);

        // Origin frame
        FrameBasePtr getOriginFramePtr();
        void setOriginFramePtr(FrameBasePtr _frame_ptr);

        // member data:
    private:
        Eigen::VectorXs data_;          ///< Motion data in form of vector mandatory
        Eigen::MatrixXs data_cov_;      ///< Motion data covariance
        MotionBuffer buffer_;           ///< Buffer of motions between this Capture and the next one.
        FrameBasePtr origin_frame_ptr_; ///< Pointer to the origin frame of the motion
};

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

inline Eigen::MatrixXs CaptureMotion::getJacobianCalib()
{
    return getBuffer().get().back().jacobian_calib_;
}

inline Eigen::MatrixXs CaptureMotion::getJacobianCalib(const TimeStamp& _ts)
{
    return getBuffer().getMotion(_ts).jacobian_calib_;
}

inline Eigen::VectorXs CaptureMotion::correctDelta(const VectorXs& _delta, const VectorXs& _delta_error)
{
    WOLF_DEBUG("WARNING: using Cartesian sum for delta correction. \nIf your deltas lie on a manifold, derive this function and implement the proper delta correction!")
    return _delta + _delta_error;
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

inline void CaptureMotion::setCalibrationPreint(const VectorXs& _calib_preint)
{
    getBuffer().setCalibrationPreint(_calib_preint);
}

inline VectorXs CaptureMotion::getDeltaPreint()
{
    return getBuffer().get().back().delta_integr_;
}

inline VectorXs CaptureMotion::getDeltaPreint(const TimeStamp& _ts)
{
    return getBuffer().getMotion(_ts).delta_integr_;
}

inline MatrixXs CaptureMotion::getDeltaPreintCov()
{
    return getBuffer().get().back().delta_integr_cov_;
}

inline MatrixXs CaptureMotion::getDeltaPreintCov(const TimeStamp& _ts)
{
    return getBuffer().getMotion(_ts).delta_integr_cov_;
}

} // namespace wolf

#endif /* SRC_CAPTURE_MOTION_H_ */
