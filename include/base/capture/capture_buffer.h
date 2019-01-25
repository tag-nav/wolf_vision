/**
 * \file capture_buffer.h
 *
 *  Created on: Jul 12, 2017
 *  \author: Jeremie Deray
 */

#ifndef _WOLF_CAPTURE_BUFFER_H_
#define _WOLF_CAPTURE_BUFFER_H_

#include "wolf.h"
#include "time_stamp.h"

#include <list>
#include <algorithm>

namespace wolf {

//using namespace Eigen;

enum class CaptureBufferPolicy : std::size_t
{
  TIME = 0,
  NUM_CAPTURES,
  ALL
};

/** \brief class for capture buffers.
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

//template <CaptureBufferPolicy policy>
class CaptureBuffer
{
public:

  CaptureBuffer(const Scalar _buffer_dt, const int _max_capture = -1);
  ~CaptureBuffer() = default;

  void push_back(const CaptureBasePtr& capture);

//  std::list<CaptureBasePtr>& get();
//  const std::list<CaptureBasePtr>& get() const;

  const CaptureBasePtr& getCapture(const TimeStamp& _ts) const;
  void getCapture(const TimeStamp& _ts, CaptureBasePtr& _motion) const;

  void remove(const CaptureBasePtr& capture);

  void clear();
//  void print();

  const TimeStamp earliest() const;
  const TimeStamp latest() const;

//private:

  int max_capture_;
  Scalar buffer_dt_;

  std::list<CaptureBasePtr> container_;
};


CaptureBuffer::CaptureBuffer(const Scalar _buffer_dt, const int _max_capture) :
  max_capture_(_max_capture), buffer_dt_(_buffer_dt)
{
  //
}

void CaptureBuffer::push_back(const CaptureBasePtr& capture)
{
  container_.push_back(capture);

  WOLF_DEBUG("Buffer dt : ", container_.back()->getTimeStamp() -
             container_.front()->getTimeStamp(), " vs ", buffer_dt_);

  while (container_.back()->getTimeStamp() -
         container_.front()->getTimeStamp() > buffer_dt_)
  {
    container_.pop_front();
  }
}

const CaptureBasePtr& CaptureBuffer::getCapture(const TimeStamp& _ts) const
{
  //assert((container_.front().ts_ <= _ts) && "Query time stamp out of buffer bounds");
  auto previous = std::find_if(container_.rbegin(), container_.rend(),
  [&](const CaptureBasePtr& c)
  {
    return c->getTimeStamp() <= _ts;
  });

  if (previous == container_.rend())
    // The time stamp is older than the buffer's oldest data.
    // We could do something here, and throw an error or something, but by now we'll return the first valid data
    previous--;

  return *previous;
}

void CaptureBuffer::getCapture(const TimeStamp& _ts, CaptureBasePtr& _capture) const
{
//  //assert((container_.front().ts_ <= _ts) && "Query time stamp out of buffer bounds");
//  auto previous = std::find_if(container_.rbegin(), container_.rend(),
//  [&](const Motion& m)
//  {
//    return c->getTimeStamp() <= _ts;
//  });

//  if (previous == container_.rend())
//    // The time stamp is older than the buffer's oldest data.
//    // We could do something here, but by now we'll return the last valid data
//    previous--;

//  _capture = *previous;

  _capture = getCapture(_ts);
}

//inline std::list<CaptureBasePtr>& CaptureBuffer::get()
//{
//  return container_;
//}

//inline const std::list<CaptureBasePtr>& CaptureBuffer::get() const
//{
//  return container_;
//}

inline void CaptureBuffer::remove(const CaptureBasePtr& capture)
{
  container_.remove(capture);
}

inline void CaptureBuffer::clear()
{
  return container_.clear();
}

inline const TimeStamp CaptureBuffer::earliest() const
{
  return (!container_.empty())? container_.front()->getTimeStamp() :
                                InvalidStamp;
}

inline const TimeStamp CaptureBuffer::latest() const
{
  return (!container_.empty())? container_.back()->getTimeStamp() :
                                InvalidStamp;
}

} // namespace wolf

#endif /* _WOLF_CAPTURE_BUFFER_H_ */
