/**
 * \file motion_buffer.h
 *
 *  Created on: Apr 14, 2016
 *      \author: jsola
 */

#ifndef SRC_MOTIONBUFFER_H_
#define SRC_MOTIONBUFFER_H_

#include <list>
#include <algorithm>

namespace wolf {


struct Motion
{
    public:
        TimeStamp ts_;                  ///< Time stamp
        Eigen::VectorXs delta_;         ///< instantaneous motion delta
        Eigen::VectorXs delta_integr_;  ///< the integrated motion or delta-integral
        Eigen::MatrixXs delta_cov_;     ///< covariance of the integrated delta
        Eigen::MatrixXs delta_integr_cov_;///< covariance of the integrated delta
        Eigen::MatrixXs jacobian_0;     ///< Jacobian of the integrated delta wrt the initial delta
        Eigen::MatrixXs jacobian_ts;    ///< Jacobian of the integrated delta wrt the current delta
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
 *   - The returned motion-integral or delta-integral is the one immediately before the query time stamp.
 *   - If the query time stamp is later than the last one in the buffer, the last motion-integral or delta-integral is returned.
 *   - It is an error if the query time stamp is earlier than the beginning of the buffer.
 */
class MotionBuffer{
    public:
        const Eigen::VectorXs& getDelta(const TimeStamp& _ts) const;
        void getDelta(const TimeStamp& _ts, Eigen::VectorXs& _delta_integr) const;
        const Motion& getMotion(const TimeStamp& _ts) const;
        void getMotion(const TimeStamp& _ts, Motion& _motion) const;
        void split(const TimeStamp& _ts, MotionBuffer& _oldest_buffer);
        std::list<Motion>& get();
        const std::list<Motion>& get() const;

    private:
        std::list<Motion> container_;
};

inline const Eigen::VectorXs& MotionBuffer::getDelta(const TimeStamp& _ts) const
{
    return getMotion(_ts).delta_integr_;
}

inline void MotionBuffer::getDelta(const TimeStamp& _ts, Eigen::VectorXs& _delta_integr) const
{
    _delta_integr = getMotion(_ts).delta_integr_;
}

inline const Motion& MotionBuffer::getMotion(const TimeStamp& _ts) const
{
    assert((container_.front().ts_ <= _ts) && "Query time stamp out of buffer bounds");
    auto previous = std::find_if(container_.rbegin(), container_.rend(), [&](const Motion& m)
    {
        return m.ts_ <= _ts;
    });
    if (previous == container_.rend())
        // The time stamp is more recent than the buffer's most recent data.
        // We could do something here, and throw an error or something, but by now we'll return the first valid data
        previous--;

    return *previous;
}

inline void MotionBuffer::getMotion(const TimeStamp& _ts, Motion& _motion) const
{
    assert((container_.front().ts_ <= _ts) && "Query time stamp out of buffer bounds");
    auto previous = std::find_if(container_.rbegin(), container_.rend(), [&](const Motion& m)
    {
        return m.ts_ <= _ts;
    });
    if (previous == container_.rend())
        // The time stamp is more recent than the buffer's most recent data.
        // We could do something here, but by now we'll return the last valid data
        previous--;

    _motion = *previous;
}


inline void MotionBuffer::split(const TimeStamp& _ts, MotionBuffer& _oldest_buffer)
{
    assert((container_.front().ts_ <= _ts) && "Query time stamp out of buffer bounds");
    auto previous = std::find_if(container_.rbegin(), container_.rend(), [&](const Motion& m)
    {
        return m.ts_ <= _ts;
    });
    if (previous == container_.rend())
    {
        // The time stamp is more recent than the buffer's most recent data:
        // return an empty buffer as the _oldest_buffer
        _oldest_buffer.get().clear();
    }
    else
    {
        // Transfer part of the buffer
        _oldest_buffer.container_.splice(_oldest_buffer.container_.begin(), container_, container_.begin(), (previous--).base());
    }
}

inline std::list<Motion>& MotionBuffer::get()
{
    return container_;
}

inline const std::list<Motion>& MotionBuffer::get() const
{
    return container_;
}

} // namespace wolf



#endif /* SRC_MOTIONBUFFER_H_ */
