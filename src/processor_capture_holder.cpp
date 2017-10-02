/**
 * \file processor_capture_holder.h
 *
 *  Created on: Jul 12, 2017
 *  \author: Jeremie Deray
 */

//Wolf includes
#include "processor_capture_holder.h"

namespace wolf {

ProcessorCaptureHolder::ProcessorCaptureHolder(const Scalar& _buffer_size) :
  ProcessorBase("CAPTURE HOLDER", _buffer_size/2.),
  buffer_(_buffer_size)
{
  //
}

void ProcessorCaptureHolder::process(CaptureBasePtr _capture_ptr)
{
  buffer_.push_back(_capture_ptr);
}

bool ProcessorCaptureHolder::keyFrameCallback(FrameBasePtr _keyframe_ptr,
                                              const Scalar& _time_tolerance)
{
  assert(_keyframe_ptr->getTrajectoryPtr() != nullptr
          && "ProcessorMotion::keyFrameCallback: key frame must be in the trajectory.");

  // get keyframe's time stamp
  const TimeStamp new_ts = _keyframe_ptr->getTimeStamp();

  // find capture whose buffer is affected by the new keyframe
  CaptureBasePtr existing_capture = findCaptureContainingTimeStamp(new_ts);

  if (existing_capture == nullptr)
  {
    WOLF_WARN("Could not find a capture at ts : ", new_ts.get());
    return false;
  }

  WOLF_DEBUG("ProcessorCaptureHolder::keyFrameCallback", _time_tolerance);
  WOLF_DEBUG("Capture of type : ", existing_capture->getType());
  WOLF_DEBUG("CaptureBuffer size : ", buffer_.container_.size());

  // add motion capture to keyframe
  if (std::abs(new_ts - existing_capture->getTimeStamp()) < _time_tolerance)
  {
    auto frame_ptr = existing_capture->getFramePtr();

    if (frame_ptr != _keyframe_ptr)
    {
      _keyframe_ptr->addCapture(existing_capture);

      //WOLF_INFO("Adding capture laser !");

      //frame_ptr->remove();
    }
    //else
      //WOLF_INFO("Capture laser already exists !");

    // Remove as we don't want duplicates
    buffer_.remove(existing_capture);

    return true;
  }
  else
  {
    WOLF_DEBUG("Capture doesn't fit dt : ",
               std::abs(new_ts - existing_capture->getTimeStamp()),
               " vs ", _time_tolerance);

    return false;
  }
}

CaptureBasePtr ProcessorCaptureHolder::findCaptureContainingTimeStamp(const TimeStamp& _ts) const
{
  WOLF_DEBUG("ProcessorCaptureHolder::findCaptureContainingTimeStamp: ts = ",
             _ts.getSeconds(), ".", _ts.getNanoSeconds());

//  auto capture_ptr = last_ptr_;
//  while (capture_ptr != nullptr)
//  {
//    // capture containing motion previous than the ts found:
//    if (buffer_.get().front()->getTimeStamp() < _ts)
//      return capture_ptr;
//    else
//    {
//      // go to the previous motion capture
//      if (capture_ptr == last_ptr_)
//        capture_ptr = std::static_pointer_cast<CaptureMotion>(origin_ptr_);
//      else if (capture_ptr->getOriginFramePtr() == nullptr)
//        return nullptr;
//      else
//      {
//        CaptureBasePtr capture_base_ptr = capture_ptr->getOriginFramePtr()->getCaptureOf(getSensorPtr());
//        if (capture_base_ptr == nullptr)
//          return nullptr;
//        else
//          capture_ptr = std::static_pointer_cast<CaptureMotion>(capture_base_ptr);
//      }
//    }
//  }

//  return capture_ptr;.

//  auto capt = buffer_.getCapture(_ts);

  /// @todo
//  WOLF_WARN("ProcessorCaptureHolder::findCaptureContainingTimeStamp "
//            "looking for stamp ",
//            _ts.get() - ((long)_ts.get()),
//            " in (",
//            buffer_.earliest().get() - ((long)buffer_.earliest().get()), ",",
//            buffer_.latest().get() - ((long)buffer_.latest().get()), ").\n",
//            "Found capture with stamp ",
//            capt->getTimeStamp().get() - ((long)capt->getTimeStamp().get()));

//  return capt;


  return buffer_.getCapture(_ts);
}

ProcessorBasePtr ProcessorCaptureHolder::create(const std::string& _unique_name,
                                                const ProcessorParamsBasePtr _params,
                                                const SensorBasePtr)
{
  ProcessorParamsCaptureHolderPtr params;

  params = std::static_pointer_cast<ProcessorParamsCaptureHolder>(_params);

  // if cast failed use default value
  if (params == nullptr)
    params = std::make_shared<ProcessorParamsCaptureHolder>();

  ProcessorCaptureHolderPtr prc_ptr = std::make_shared<ProcessorCaptureHolder>(params->buffer_size_);
  prc_ptr->setName(_unique_name);

  return prc_ptr;
}

} // namespace wolf

// Register in the ProcessorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("CAPTURE HOLDER", ProcessorCaptureHolder)
} // namespace wolf
