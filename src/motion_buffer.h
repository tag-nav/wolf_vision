/**
 * \file motion_buffer.h
 *
 *  Created on: Apr 14, 2016
 *      \author: jsola
 */

#ifndef SRC_MOTIONBUFFER_H_
#define SRC_MOTIONBUFFER_H_

#include "wolf.h"
#include "time_stamp.h"

#include <list>
#include <algorithm>

namespace wolf {

using namespace Eigen;

struct Motion
{
    public:
        Size delta_size_, cov_size_;
        TimeStamp ts_;                          ///< Time stamp
        Eigen::VectorXs delta_;                 ///< instantaneous motion delta
        Eigen::VectorXs delta_integr_;          ///< the integrated motion or delta-integral
        Eigen::MatrixXs jacobian_delta_;        ///< Jacobian of the integration wrt delta_
        Eigen::MatrixXs jacobian_delta_integr_; ///< Jacobian of the integration wrt delta_integr_
        Eigen::MatrixXs delta_cov_;             ///< covariance of the instantaneous delta
//        Eigen::MatrixXs delta_integr_cov_;      ///< covariance of the integrated delta
    public:
        Motion();
        Motion(const TimeStamp& _ts, Size _delta_size = 0, Size _cov_size = 0);
//        Motion(const TimeStamp& _ts, const VectorXs& _delta, const VectorXs& _delta_int, Size _cov_size);
        Motion(const TimeStamp& _ts, const VectorXs& _delta, const VectorXs& _delta_int, const MatrixXs& _jac_delta, const MatrixXs& _jac_delta_int, const MatrixXs& _delta_cov);
        ~Motion();
        void resize(Size ds, Size dcs);
        void resize(Size ds);

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
        Size delta_size_, cov_size_;
        MotionBuffer(Size _delta_size, Size _cov_size);
        std::list<Motion>& get();
        const std::list<Motion>& get() const;
        const Motion& getMotion(const TimeStamp& _ts) const;
        void getMotion(const TimeStamp& _ts, Motion& _motion) const;
        const Eigen::VectorXs& getDelta(const TimeStamp& _ts) const;
        void getDelta(const TimeStamp& _ts, Eigen::VectorXs& _delta_integr) const;
        void split(const TimeStamp& _ts, MotionBuffer& _oldest_buffer);

    private:
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

inline const Eigen::VectorXs& MotionBuffer::getDelta(const TimeStamp& _ts) const
{
    return getMotion(_ts).delta_integr_;
}

inline void MotionBuffer::getDelta(const TimeStamp& _ts, Eigen::VectorXs& _delta_integr) const
{
    _delta_integr = getMotion(_ts).delta_integr_;
}

} // namespace wolf

#endif /* SRC_MOTIONBUFFER_H_ */
