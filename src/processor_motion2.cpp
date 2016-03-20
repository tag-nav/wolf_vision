/**
 * \file processor_motion2.cpp
 *
 *  Created on: 15/03/2016
 *      \author: jsola
 */

#include "processor_motion2.h"

ProcessorMotion2::ProcessorMotion2(ProcessorType _tp, WolfScalar _dt, size_t _state_size, size_t _delta_size,
                                   size_t _data_size, size_t _noise_size) :
        ProcessorBase(_tp), dt_(_dt), x_size_(_state_size), delta_size_(_delta_size), data_size_(_data_size), noise_size_(
                _noise_size), origin_ptr_(nullptr), last_ptr_(nullptr), x_origin_(_state_size), x_(_state_size), x_t_(
                _state_size), ts_(0), delta_(_delta_size), delta_integrated_(_delta_size), data_(_data_size)
{
    //
}

ProcessorMotion2::~ProcessorMotion2()
{
    //
}

void ProcessorMotion2::process(CaptureBase* _incoming_ptr)
{
    CaptureMotion2* incoming_ptr = (CaptureMotion2*)(_incoming_ptr);
//    incoming_ptr->getBufferPtr()->setDt(dt_);
    integrate(incoming_ptr);
    if (voteForKeyFrame() && permittedKeyFrame())
    {
        // TODO:
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

    origin_ptr_ = _origin_ptr;
    last_ptr_ = _origin_ptr;
    x_origin_ = _origin_ptr->getFramePtr()->getState();
    delta_integrated_ = Eigen::VectorXs::Zero(delta_size_);
    extractData(_origin_ptr);
    getBufferPtr()->clear();
    getBufferPtr()->setDt(dt_);
    getBufferPtr()->pushBack(_origin_ptr->getTimeStamp(), delta_integrated_);
}
