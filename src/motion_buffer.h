/**
 * \file MotionBuffer.h
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
        Eigen::VectorXs delta_;         ///< the integrated motion or delta-integral
        Eigen::MatrixXs jacobian_0;     ///< Jacobian of the integrated delta wrt the initial delta
        Eigen::MatrixXs jacobian_ts;    ///< Jacobian of the integrated delta wrt the current delta
        Eigen::MatrixXs covariance_;    ///< covariance of the integrated delta
}; ///< One instance of the buffered data, corresponding to a particular time stamp.


class MotionBuffer{
    public:
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
         * The buffer can be queried for delta-integrals corresponding to a provided time stamp, with the following rules:
         *   - The returned delta-integral is the one immediately before the query time stamp.
         *   - If the query time stamp is later than the last one in the buffer, the last delta-integral is returned.
         *   - It is an error if the query time stamp is earlier than the beginning of the buffer.
         */
        MotionBuffer(){}
        void pushBack(const Motion _motion){container_.push_back(_motion);}
        void pushBack(const TimeStamp _ts, const Eigen::VectorXs& _delta) { container_.push_back(Motion( {_ts, _delta})); }
        void clear(){container_.clear();}
        unsigned int size() const {return container_.size();}
        bool empty() const {return container_.empty();}
        const TimeStamp& getTimeStamp() const {return container_.back().ts_;}
        const Eigen::VectorXs& getDelta() const {return container_.back().delta_;}
        const Eigen::VectorXs& getDelta(const TimeStamp& _ts) const {
            return getMotion(_ts).delta_;
        }
        const Motion& getMotion() const {return container_.back();}
        const Motion& getMotion(const TimeStamp& _ts) const;

    private:
        std::list<Motion> container_;
};


inline const Motion& MotionBuffer::getMotion(const TimeStamp& _ts) const
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

    return *previous;

//    Motion copy = *previous;
//
//    return copy;
}

} // namespace wolf



#endif /* SRC_MOTIONBUFFER_H_ */
