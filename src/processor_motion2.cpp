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
    integrate(_incoming_ptr);

    if (voteForKeyFrame() && permittedKeyFrame())
    {
        // Make KeyFrame
//        makeKeyFrame(incoming_ptr_);
        // Reset the Tracker
//        reset();
    }
    else
    {   // We did not create a KeyFrame:
        // 2.b. Update the tracker's last and incoming pointers one step ahead
        advance();
    }
}

void ProcessorMotion2::init(CaptureMotion2* _origin_ptr)
{
    origin_ptr_ = _origin_ptr;
    last_ptr_ = _origin_ptr;
    x_origin_ = _origin_ptr->getFramePtr()->getState();
    extractData(_origin_ptr);
    bufferPtr()->clear();
    motion_.ts_ = ts_origin_;
    motion_.Dx_.setZero();
    bufferPtr()->push_back(motion_);
}

void ProcessorMotion2::deltaState(const TimeStamp& _t1, const TimeStamp& _t2, Eigen::VectorXs& _Delta)
{
    unsigned int i1 = index(_t1);
    if (i1 == 0)
        _Delta = bufferPtr()->at(index(_t2)).Dx_;
    else
        deltaMinusDelta(bufferPtr()->at(index(_t2)).Dx_, bufferPtr()->at(i1).Dx_, _Delta);
}
