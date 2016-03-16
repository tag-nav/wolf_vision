/**
 * \file processor_motion2.cpp
 *
 *  Created on: 15/03/2016
 *      \author: jsola
 */

#include "processor_motion2.h"

ProcessorMotion2::ProcessorMotion2(ProcessorType _tp, size_t _state_size, size_t _delta_size, size_t _data_size,
                                   size_t _noise_size, WolfScalar _dt) :
        ProcessorBase(_tp), x_size_(_state_size), dx_size_(_delta_size), data_size_(_data_size), noise_size_(
                _noise_size), origin_ptr_(nullptr), dt_(_dt), ts_(0)
{
    buffer_.clear(); // just to be explicit; probably not needed.
}

ProcessorMotion2::~ProcessorMotion2()
{
    // TODO Auto-generated destructor stub
}

void ProcessorMotion2::process(CaptureBase* _incoming_ptr)
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

void ProcessorMotion2::init(const CaptureBase* _origin_ptr)
{
    x_origin_ = _origin_ptr->getFramePtr()->getState();
    extractData(_origin_ptr);
    buffer_.clear();
    motion_.ts_ = ts_origin_;
    motion_.dx_ = dx_;
    motion_.Dx_.setZero();
    buffer_.push_back(motion_);
}

void ProcessorMotion2::makeKeyFrame(const TimeStamp& _t)
{
}

void ProcessorMotion2::update()
{
    x_origin_ = origin_ptr_->getFramePtr()->getState();
}

void ProcessorMotion2::reset(const TimeStamp& _t)
{
    // This is the index of the new key-frame
}

void ProcessorMotion2::advance()
{
    // Nothing to do?
}

void ProcessorMotion2::state(const TimeStamp& _t, Eigen::VectorXs& _x)
{
    update();
    xPlusDelta(x_origin_, buffer_.at(index(_t)).Dx_, _x);
}

void ProcessorMotion2::deltaState(const TimeStamp& _t1, const TimeStamp& _t2, Eigen::VectorXs& _Delta)
{
    unsigned int i1 = index(_t1);
    if (i1 == 0)
        _Delta = buffer_.at(index(_t2)).Dx_;
    else
        deltaMinusDelta(buffer_.at(index(_t2)).Dx_, buffer_.at(i1).Dx_, _Delta);
}

void ProcessorMotion2::sumDeltas(const CaptureMotion* _cap1_ptr, const CaptureMotion* _cap2_ptr,
                                 Eigen::VectorXs& _delta_1_2) const
{
    // TODO: implement with external data from given captures. This data should be in the form of buffers of the same type.
}

unsigned int ProcessorMotion2::index(const TimeStamp& _t)
{

    // Assume dt is constant and known, and exists in dt_
    return (buffer_.back().ts_ - buffer_.front().ts_) / dt_ + 0.5; // we rounded to the nearest entry in the buffer
}

void ProcessorMotion2::integrate(CaptureBase* _incoming_ptr)
{
    // First get data and push it into buffer
    extractData(_incoming_ptr);
    motion_.ts_ = ts_;
    motion_.dx_ = dx_;
    deltaPlusDelta(buffer_.back().Dx_, dx_, motion_.Dx_);
    buffer_.push_back(motion_);
}
