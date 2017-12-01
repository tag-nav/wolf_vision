/*
 * ConstraintBearing.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#include "constraint_bearing.h"

namespace wolf
{

ConstraintBearing::ConstraintBearing(const LandmarkBasePtr& _landmark_other_ptr,
                                     const ProcessorBasePtr& _processor_ptr,
                                     bool _apply_loss_function,
                                     ConstraintStatus _status) :
        ConstraintAutodiff<ConstraintBearing, 1, 2, 1, 2>(CTR_BEARING_2D,
                                                          nullptr,
                                                          nullptr,
                                                          nullptr,
                                                          _landmark_other_ptr,
                                                          _processor_ptr,
                                                          _apply_loss_function,
                                                          _status,
                                                          getCapturePtr()->getFramePtr()->getPPtr(),
                                                          getCapturePtr()->getFramePtr()->getOPtr(),
                                                          _landmark_other_ptr->getPPtr())
{
    //
}

ConstraintBearing::~ConstraintBearing()
{
    //
}

} /* namespace wolf */
