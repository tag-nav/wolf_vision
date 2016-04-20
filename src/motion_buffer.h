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
        Eigen::MatrixXs covariance_;    ///< covariance of the integrated delta
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
        void pushBack(const Motion _motion);
        void pushBack(const TimeStamp _ts, const Eigen::VectorXs& _delta, const Eigen::MatrixXs& _cov = Eigen::MatrixXs(0,0), const Eigen::MatrixXs& _J_0 = Eigen::MatrixXs(0,0), const Eigen::MatrixXs& _J_t = Eigen::MatrixXs(0,0));
        void clear();
        unsigned int size() const;
        bool empty() const;
        const TimeStamp& getTimeStamp() const;
        const Eigen::VectorXs& getDelta() const;
        const Eigen::VectorXs& getDelta(const TimeStamp& _ts) const;
        const Motion& getMotion() const;
        const Motion& getMotion(const TimeStamp& _ts) const;
        void split(TimeStamp& _ts, MotionBuffer& _oldest_part);

    private:
        std::list<Motion> container_;
};


inline void MotionBuffer::pushBack(const Motion _motion)
{
    container_.push_back(_motion);
}

inline void MotionBuffer::pushBack(const TimeStamp _ts, const Eigen::VectorXs& _delta, const Eigen::MatrixXs& _cov, const Eigen::MatrixXs& _J_0, const Eigen::MatrixXs& _J_t)
{
    container_.push_back(Motion({_ts, _delta, _cov, _J_0, _J_t}));
}

inline void MotionBuffer::clear()
{
    container_.clear();
}

inline unsigned int MotionBuffer::size() const
{
    return container_.size();
}

inline bool MotionBuffer::empty() const
{
    return container_.empty();
}

inline const TimeStamp& MotionBuffer::getTimeStamp() const
{
    return container_.back().ts_;
}

inline const Eigen::VectorXs& MotionBuffer::getDelta() const
{
    return container_.back().delta_;
}

inline const Eigen::VectorXs& MotionBuffer::getDelta(const TimeStamp& _ts) const
{
    return getMotion(_ts).delta_;
}

inline const Motion& MotionBuffer::getMotion() const
{
    return container_.back();
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
        // We could do something here, but by now we'll return the last valid data
        previous--;

    return *previous;
}

inline void MotionBuffer::split(TimeStamp& _ts, MotionBuffer& _oldest_part)
{
    // TODO: Do the split. Look at this code:
    //
    //    std::list<std::string> lst1 = { "1", "2", "3", "4" };
    //
    //    for (const auto &s : lst1 ) std::cout << s << ' ';
    //    std::cout << std::endl;
    //
    //    std::cout << std::endl;
    //
    //    std::list<std::string> lst2;
    //    lst2.splice( lst2.begin(),
    //                 lst1,
    //                 lst1.begin(),
    //                 std::next( lst1.begin(), lst1.size() / 2 ) );
    //
    //    for (const auto &s : lst2 ) std::cout << s << ' ';
    //    std::cout << std::endl;
    //
    //    for (const auto &s : lst1 ) std::cout << s << ' ';
    //    std::cout << std::endl;

}

} // namespace wolf



#endif /* SRC_MOTIONBUFFER_H_ */
