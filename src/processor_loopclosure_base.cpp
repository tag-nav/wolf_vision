/**
 * \file processor_loop_closure.h
 *
 *  Created on: Aug 10, 2017
 *      \author: Tessa Johanna
 */

#include "processor_loopclosure_base.h"

namespace wolf
{

ProcessorLoopClosureBase::ProcessorLoopClosureBase(const std::string& _type, const Scalar _time_tolerance):
  ProcessorBase(_type, _time_tolerance)
{
  //
}

//##############################################################################
void ProcessorLoopClosureBase::process(CaptureBasePtr _incoming_ptr)
{
  // clear all previous data from vector
  loop_closure_candidates.clear();
  close_candidates.clear();

  // the pre-process, if necessary, is implemented in the derived classes
  preProcess();

  findCandidates(_incoming_ptr);

  confirmLoopClosure();

  WOLF_DEBUG("ProcessorLoopClosureBase::process found ",
             loop_closure_candidates.size(), " candidates found.");

  // the post-process, if necessary, is implemented in the derived classes
  postProcess();
}

//##############################################################################
bool ProcessorLoopClosureBase::keyFrameCallback(FrameBasePtr /*_keyframe_ptr*/,
                                                const Scalar& /*_time_tol_other*/)
{
  return false;
}

}// namespace wolf

