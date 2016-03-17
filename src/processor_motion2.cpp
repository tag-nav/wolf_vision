/**
 * \file processor_motion2.cpp
 *
 *  Created on: 15/03/2016
 *      \author: jsola
 */

#include "processor_motion2.h"

ProcessorMotion2::ProcessorMotion2(ProcessorType _tp, size_t _state_size, size_t _delta_size, size_t _data_size,
                                   size_t _noise_size, WolfScalar _dt) :
        ProcessorBase(_tp), x_size_(_state_size), delta_size_(_delta_size), data_size_(_data_size),
        noise_size_(_noise_size), origin_ptr_(nullptr), last_ptr_(nullptr), dt_(_dt), ts_(0)
{
    //
}

ProcessorMotion2::~ProcessorMotion2()
{
    // TODO Auto-generated destructor stub
}

void ProcessorMotion2::process(CaptureMotion2* _incoming_ptr)
{
    _incoming_ptr->getBufferPtr()->setDt(dt_);
    integrate(_incoming_ptr);
    if (voteForKeyFrame() && permittedKeyFrame())
    {
        // Make KeyFrame
        //        makeKeyFrame(incoming_ptr_);
        // Reset the Tracker
        //        reset();
    }
}

void ProcessorMotion2::init(CaptureMotion2* _origin_ptr)
{
    //TODO: This fcn needs to change:
    // input: framebase: this is a Keyframe
    // create a new non-key Frame and make it last_
    // first delta in buffer must be zero
    // we do not need to extract data from any Capture

//    origin_ptr_ = _origin_ptr;
//    last_ptr_ = _origin_ptr;
//    x_origin_ = _origin_ptr->getFramePtr()->getState();
//    extractData(_origin_ptr);
//    data2delta();
//    getBufferPtr()->clear();
//    getBufferPtr()->addMotion(ts_origin_, delta_integrated_);
}
