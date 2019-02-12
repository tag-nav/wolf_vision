/**
 * \file motion_buffer.h
 *
 *  Created on: Apr 14, 2016
 *      \author: jsola
 */

#ifndef SRC_MOTIONBUFFER_H_
#define SRC_MOTIONBUFFER_H_

#include "base/wolf.h"
#include "base/time_stamp.h"

#include <list>
#include <algorithm>

namespace wolf {

using namespace Eigen;

struct Motion
{
    public:
        SizeEigen data_size_, delta_size_, cov_size_, calib_size_;
        TimeStamp ts_;                          ///< Time stamp
        Eigen::VectorXs data_;                  ///< instantaneous motion data
        Eigen::MatrixXs data_cov_;              ///< covariance of the instantaneous data
        Eigen::VectorXs delta_;                 ///< instantaneous motion delta
        Eigen::MatrixXs delta_cov_;             ///< covariance of the instantaneous delta
        Eigen::VectorXs delta_integr_;          ///< the integrated motion or delta-integral
        Eigen::MatrixXs delta_integr_cov_;      ///< covariance of the integrated delta
        Eigen::MatrixXs jacobian_delta_;        ///< Jacobian of the integration wrt delta_
        Eigen::MatrixXs jacobian_delta_integr_; ///< Jacobian of the integration wrt delta_integr_
        Eigen::MatrixXs jacobian_calib_;        ///< Jacobian of delta_integr wrt extra states (TBD by the derived processors)
    public:
        Motion() = delete; // completely delete unpredictable stuff like this
        Motion(const TimeStamp& _ts, SizeEigen _data_size, SizeEigen _delta_size, SizeEigen _cov_size, SizeEigen _calib_size);
        Motion(const TimeStamp& _ts,
               const VectorXs& _data,
               const MatrixXs& _data_cov,
               const VectorXs& _delta,
               const MatrixXs& _delta_cov,
               const VectorXs& _delta_int,
               const MatrixXs& _delta_integr_cov,
               const MatrixXs& _jac_delta,
               const MatrixXs& _jac_delta_int,
               const MatrixXs& _jacobian_calib);// = MatrixXs::Zero(0,0));
        ~Motion();
    private:
        void resize(SizeEigen _data_s, SizeEigen _delta_s, SizeEigen _delta_cov_s, SizeEigen _calib_s);

}; ///< One instance of the buffered data, corresponding to a particular time stamp.

/** \brief class for motion buffers.
 *
 * It consists of a buffer of pre-integrated motions (aka. delta-integrals) that is being filled
 * by the motion processors (deriving from ProcessorMotion).
 * Each delta-integral is accompanied by a time stamp, a Jacobian and a covariances matrix.
 *
 * This buffer contains the time stamp and delta-integrals:
 *  - since the last key-Frame
 *  - until the frame of this capture.
 *
 * The buffer can be queried for motion-integrals and delta-integrals corresponding to a provided time stamp,
 * with the following rules:
 *   - If the query time stamp is later than the last one in the buffer,
 *     the last motion-integral or delta-integral is returned.
 *   - If the query time stamp is earlier than the beginning of the buffer,
 *     the earliest Motion or Delta is returned.
 *   - If the query time stamp matches one time stamp in the buffer exactly,
 *     the returned motion-integral or delta-integral is the one of the queried time stamp.
 *   - If the query time stamp does not match any time stamp in the buffer,
 *     the returned motion-integral or delta-integral is the one immediately before the query time stamp.
 */
class MotionBuffer{
    public:
        SizeEigen data_size_, delta_size_, cov_size_, calib_size_;
        MotionBuffer() = delete;
        MotionBuffer(SizeEigen _data_size, SizeEigen _delta_size, SizeEigen _cov_size, SizeEigen _calib_size);
        std::list<Motion>& get();
        const std::list<Motion>& get() const;
        const VectorXs& getCalibrationPreint() const;
        void setCalibrationPreint(const VectorXs& _calib_preint);
        const Motion& getMotion(const TimeStamp& _ts) const;
        void getMotion(const TimeStamp& _ts, Motion& _motion) const;
        void split(const TimeStamp& _ts, MotionBuffer& _oldest_buffer);
//        MatrixXs integrateCovariance() const; // Integrate all buffer
//        MatrixXs integrateCovariance(const TimeStamp& _ts) const; // Integrate up to time stamp (included)
//        MatrixXs integrateCovariance(const TimeStamp& _ts_1, const TimeStamp _ts_2) const; // integrate between time stamps (both included)
        void print(bool show_data = 0, bool show_delta = 0, bool show_delta_int = 0, bool show_jacs = 0);

    private:
        VectorXs calib_preint_;         ///< value of the calibration params during pre-integration
        std::list<Motion> container_;
};

inline std::list<Motion>& MotionBuffer::get()
{
    return container_;
}

inline const std::list<Motion>& MotionBuffer::get() const
{
    return container_;
}

inline const VectorXs& MotionBuffer::getCalibrationPreint() const
{
    return calib_preint_;
}

inline void MotionBuffer::setCalibrationPreint(const VectorXs& _calib_preint)
{
    assert(_calib_preint.size() == calib_size_ && "Wrong size of calibration parameters");

    calib_preint_ = _calib_preint;
}

} // namespace wolf

#endif /* SRC_MOTIONBUFFER_H_ */
