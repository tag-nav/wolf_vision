/**
 * \file processor_capture_holder.h
 *
 *  Created on: Jul 12, 2017
 *  \author: Jeremie Deray
 */

#ifndef _WOLF_PROCESSOR_CAPTURE_HOLDER_H_
#define _WOLF_PROCESSOR_CAPTURE_HOLDER_H_

//Wolf includes
#include "base/processor/processor_base.h"
#include "base/capture/capture_base.h"
#include "base/capture/capture_buffer.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(ProcessorCaptureHolder);
WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsCaptureHolder);

/**
 * \brief ProcessorParamsCaptureHolder
 */
struct ProcessorParamsCaptureHolder : public ProcessorParamsBase
{
  Scalar buffer_size = 30;
};

/**
 * \brief ProcessorCaptureHolder
 */
class ProcessorCaptureHolder : public ProcessorBase
{
public:

  ProcessorCaptureHolder(ProcessorParamsCaptureHolderPtr _params_capture_holder);
  virtual ~ProcessorCaptureHolder() = default;
  virtual void configure(SensorBasePtr _sensor) override { };

  virtual void process(CaptureBasePtr _capture_ptr) override;

  /** \brief Vote for KeyFrame generation
   *
   * If a KeyFrame criterion is validated, this function returns true,
   * meaning that it wants to create a KeyFrame at the \b last Capture.
   *
   * WARNING! This function only votes! It does not create KeyFrames!
   */
  virtual bool voteForKeyFrame() override { return false; }

  virtual void keyFrameCallback(FrameBasePtr _keyframe_ptr,
                                const Scalar& _time_tolerance) override;

  /**
   * \brief Finds the capture that contains the closest previous motion of _ts
   * \return a pointer to the capture (if it exists) or a nullptr (otherwise)
   */
  CaptureBasePtr findCaptureContainingTimeStamp(const TimeStamp& _ts) const;

protected:

  ProcessorParamsCaptureHolderPtr params_capture_holder_;
  CaptureBuffer buffer_;

public:

  static ProcessorBasePtr create(const std::string& _unique_name,
                                 const ProcessorParamsBasePtr _params,
                                 const SensorBasePtr sensor_ptr = nullptr);
};

} // namespace wolf

#endif // _WOLF_PROCESSOR_CAPTURE_HOLDER_H_
