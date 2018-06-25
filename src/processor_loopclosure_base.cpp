/**
 * \file processor_loop_closure.h
 *
 *  Created on: Aug 10, 2017
 *      \author: Tessa Johanna
 */

#include "processor_loopclosure_base.h"

namespace wolf
{

ProcessorLoopClosureBase::ProcessorLoopClosureBase(const std::string& _type, ProcessorParamsLoopClosurePtr _params_loop_closure):
  ProcessorBase(_type, _params_loop_closure),
  params_loop_closure_(_params_loop_closure)
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
void ProcessorLoopClosureBase::keyFrameCallback(FrameBasePtr /*_keyframe_ptr*/,
                                                const Scalar& /*_time_tol_other*/)
{
    //
}

}// namespace wolf

