#ifndef _WOLF_PROCESSOR_LOOPCLOSURE_BASE_H
#define _WOLF_PROCESSOR_LOOPCLOSURE_BASE_H

// Wolf related headers
#include "processor_base.h"

namespace wolf{

struct ProcessorParamsLoopClosure : public ProcessorParamsBase
{
  virtual ~ProcessorParamsLoopClosure() = default;

  // add neccesery parameters for loop closure initialisation here and initialize
  // them in constructor
};

/** \brief General loop closure processor
 *
 * This is an abstract class.
 *
 * It establishes constraints XXX
 *
 * Should you need extra functionality for your derived types,
 * you can overload these two methods,
 *
 *   -  preProcess() { }
 *   -  postProcess() { }
 *
 * which are called at the beginning and at the end of process() respectively.
 */

class ProcessorLoopClosureBase : public ProcessorBase
{
protected:

  // Frames that are possible loop closure candidates according to
  // findLoopClosure()
  std::vector<FrameBasePtr> loop_closure_candidates;

  // Frames that are possible loop closure candidates according to
  // findLoopClosure(), but are too recent in the timeline, aka still in a
  // 'buffer zone'. This vector will capture the frames that were set just before
  // the last keyframe.
  std::vector<FrameBasePtr> close_candidates;

public:

  ProcessorLoopClosureBase(const std::string& _type, const Scalar _time_tolerance);

  virtual ~ProcessorLoopClosureBase() = default;

  /** \brief Full processing of an incoming Capture.
     *
     * Usually you do not need to overload this method in derived classes.
     * Overload it only if you want to alter this algorithm.
     */
  virtual void process(CaptureBasePtr _incoming_ptr) override;

  virtual bool keyFrameCallback(FrameBasePtr _keyframe_ptr,
                                const Scalar& _time_tol_other) ;

  const std::vector<FrameBasePtr>& getCandidates() const noexcept;

  const std::vector<FrameBasePtr>& getCloseCandidates() const noexcept;

protected:

  /** Pre-process incoming Capture
   *
   * This is called by process() just after assigning incoming_ptr_ to a valid Capture.
   *
   * Overload this function to prepare stuff on derived classes.
   *
   * Typical uses of prePrecess() are:
   *   - casting base types to derived types
   *   - initializing counters, flags, or any derived variables
   *   - initializing algorithms needed for processing the derived data
   */
  virtual void preProcess() { }

  /** Post-process
   *
   * This is called by process() after finishing the processing algorithm.
   *
   * Overload this function to post-process stuff on derived classes.
   *
   * Typical uses of postPrecess() are:
   *   - resetting and/or clearing variables and/or algorithms at the end of processing
   *   - drawing / printing / logging the results of the processing
   */
  virtual void postProcess() { }


  /** Find a loop closure between incoming data and all keyframe data
   *
   * This is called by process() .
   *
   * Overload this function in derived classes to find loop closure.
   */
  virtual bool findCandidates(const CaptureBasePtr& _incoming_ptr) = 0;

  /** perform a validation among the found possible loop closures to confirm
   * or dismiss them based on available data
   *
   * This is called by process() .
   *
   * Overload this function in derived classes to confirm loop closure.
   */
  virtual bool confirmLoopClosure() = 0;

  /** \brief Vote for KeyFrame generation
   *
   * If a KeyFrame criterion is validated, this function returns true,
   * meaning that it wants to create a KeyFrame at the \b last Capture.
   *
   * WARNING! This function only votes! It does not create KeyFrames!
   */
  virtual bool voteForKeyFrame() override = 0;
};

inline const std::vector<FrameBasePtr>&
ProcessorLoopClosureBase::getCandidates() const noexcept
{
  return loop_closure_candidates;
}

inline const std::vector<FrameBasePtr>&
ProcessorLoopClosureBase::getCloseCandidates() const noexcept
{
  return close_candidates;
}

} // namespace wolf

#endif /* _WOLF_PROCESSOR_LOOPCLOSURE_BASE_H */
