/**
 * \file constraint_epipolar.cpp
 *
 *  Created on: May 8, 2016
 *      \author: jsola
 */

#include "constraint_epipolar.h"


//===================================================================================================================
// Register in the SensorFactory
#include "constraint_factory.h"
namespace wolf {
namespace
{
const bool registered_ctr_epipolar = ConstraintFactory::get()->registerCreator("EPIPOLAR", ConstraintEpipolar::create);
}
} // namespace wolf
