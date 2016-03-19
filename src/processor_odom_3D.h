/**
 * \file processor_odom_3D.h
 *
 *  Created on: Mar 18, 2016
 *      \author: jsola
 */

#ifndef SRC_PROCESSOR_ODOM_3D_H_
#define SRC_PROCESSOR_ODOM_3D_H_

#include "processor_motion2.h"

// This is the class under test!
class ProcessorOdom3d : public ProcessorMotion2
{
    public:
        ProcessorOdom3d(WolfScalar _delta_t) : ProcessorMotion2(PRC_ODOM_3D, _delta_t, 7, 6, 6, 6) {}
        virtual ~ProcessorOdom3d(){}
        virtual void data2delta();

    private:
        void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta);
        void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2);
        virtual void deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                     Eigen::VectorXs& _delta2_minus_delta1);

    private:
        Eigen::Vector3s pos1_, delta_pos_;
        Eigen::Quaternions quat1_, delta_quat_;
};


inline void ProcessorOdom3d::data2delta()
{
    // Trivial implementation
    delta_ = data_;
}


void v2q(const Eigen::VectorXs& _v, Eigen::Quaternions& _q){
    WolfScalar angle = _v.norm();
    if (angle < WolfConstants::EPS)
        _q = Eigen::Quaternions::Identity();
    else
    {
        _q = Eigen::Quaternions(Eigen::AngleAxiss(angle, _v/angle));
    }
}
void q2v(const Eigen::Quaternions& _q, Eigen::VectorXs& _v){
    Eigen::AngleAxiss aa = Eigen::AngleAxiss(_q);
    _v = aa.axis() * aa.angle();
}


// Compose a frame with a delta frame and give a frame (sizes 7, 6, 7 respectively)
inline void ProcessorOdom3d::xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta)
{
    assert(_x.size() == 7 && "Wrong _x vector size");
    assert(_delta.size() == 6 && "Wrong _delta vector size");
    assert(_x_plus_delta.size() == 7 && "Wrong _x_plus_delta vector size");

    pos1_ = _x.head(3);
    quat1_ = Eigen::Map<const Eigen::Quaternions>(&_x(3));
    delta_pos_ = _delta.head(3);
    v2q(_delta.tail(3),delta_quat_);

    _x_plus_delta.head(3) = pos1_ + quat1_*delta_pos_;
    _x_plus_delta.tail(4) = (quat1_*delta_quat_).coeffs();
}

// Compose two delta-frames and give a delta-frame (all of size 6)
inline void ProcessorOdom3d::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2)
{
    assert(_delta1.size() == 6 && "Wrong _delta1 vector size");
    assert(_delta2.size() == 6 && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == 6 && "Wrong _delta1_plus_delta2 vector size");

    pos1_ = _delta1.head(3);
    v2q(_delta1.tail(3), quat1_);
    delta_pos_ = _delta2.head(3);
    v2q(_delta2.tail(3),delta_quat_);

    Eigen::VectorXs v(3);
    q2v(quat1_*delta_quat_,v);

    _delta1_plus_delta2.head(3) = pos1_ + quat1_*delta_pos_;
    _delta1_plus_delta2.tail(3) = v;
}

inline void ProcessorOdom3d::deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                             Eigen::VectorXs& _delta2_minus_delta1)
{
    assert(_delta1.size() == 6 && "Wrong _delta1 vector size");
    assert(_delta2.size() == 6 && "Wrong _delta2 vector size");
    assert(_delta2_minus_delta1.size() == 6 && "Wrong _delta2_minus_delta1 vector size");

    pos1_ = _delta1.head(3);
    quat1_ = Eigen::Map<const Eigen::Quaternions>(&_delta1(3));
    delta_pos_ = _delta2.head(3);
    v2q(_delta2.tail(3),delta_quat_);

    Eigen::VectorXs v(3);
    q2v(quat1_*delta_quat_,v);

    _delta2_minus_delta1.head(3) = pos1_ + quat1_*delta_pos_;
    _delta2_minus_delta1.tail(3) = v;
}

#endif /* SRC_PROCESSOR_ODOM_3D_H_ */
