/*
 * ProcessorTracker.cpp
 *
 *  Created on: Apr 7, 2016
 *      Author: jvallve
 */

#include "processor_tracker.h"

ProcessorTracker::ProcessorTracker(ProcessorType _tp) :
    ProcessorBase(_tp),
    origin_ptr_(nullptr),
    last_ptr_(nullptr),
    incoming_ptr_(nullptr)
{
    // TODO Auto-generated constructor stub

}

ProcessorTracker::~ProcessorTracker()
{
    // TODO Auto-generated destructor stub
}



