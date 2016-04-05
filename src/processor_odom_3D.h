/**
 * \file processor_odom_3D.h
 *
 *  Created on: Mar 18, 2016
 *      \author: jsola
 */

#ifndef SRC_PROCESSOR_ODOM_3D_H_
#define SRC_PROCESSOR_ODOM_3D_H_

#include "processor_motion2.h"

/**\brief The Motion Delta type
 *
 * The motion delta, as a composite struct containing position increment and orientation quaternion increment.
 */
struct Odom3dDelta{
        Eigen::Vector3s dp;    ///< Position increment
        Eigen::Quaternions dq; ///< Orientation increment as a quaternion
        Odom3dDelta() : dp(0,0,0), dq(1,0,0,0) {};
        Odom3dDelta(const Eigen::Vector3s& _dp, const Eigen::Quaternions& _dq) : dp(_dp), dq(_dq) {};
        void set(Eigen::Vector3s _dp, Eigen::Quaternions _dq){dp = _dp; dq = _dq;};
        void setZero() {dp.setZero(); dq.setIdentity();}
        static Odom3dDelta Zero() {return Odom3dDelta();}
};



class ProcessorOdom3d : public ProcessorMotion2<Odom3dDelta>
{
    public:
        ProcessorOdom3d(WolfScalar _delta_t) : ProcessorMotion2(PRC_ODOM_3D, _delta_t, 7, 6), quat1_(nullptr) {}
        virtual ~ProcessorOdom3d(){}
        virtual void data2delta(const Eigen::VectorXs& _data, const WolfScalar _dt, Odom3dDelta& _delta);

    private:
        void xPlusDelta(const Eigen::VectorXs& _x, const Odom3dDelta& _delta, Eigen::VectorXs& _x_plus_delta);
        void deltaPlusDelta(const Odom3dDelta& _delta1, const Odom3dDelta& _delta2, Odom3dDelta& _delta1_plus_delta2);
        virtual void deltaMinusDelta(const Odom3dDelta& _delta1, const Odom3dDelta& _delta2,
                                     Odom3dDelta& _delta2_minus_delta1);
        Odom3dDelta deltaZero() const;

    private:
        Eigen::Map<const Eigen::Quaternions> quat1_;
};


inline void ProcessorOdom3d::data2delta(const Eigen::VectorXs& _data, const WolfScalar _dt, Odom3dDelta& _delta)
{
    // Trivial implementation
    _delta.dp = _data.head(3);
    Eigen::v2q(_data.tail(3), _delta.dq);
}



// Compose a frame with a delta frame and give a frame
inline void ProcessorOdom3d::xPlusDelta(const Eigen::VectorXs& _x, const Odom3dDelta& _delta, Eigen::VectorXs& _x_plus_delta)
{
    assert(_x.size() == 7 && "Wrong _x vector size");
    assert(_x_plus_delta.size() == 7 && "Wrong _x_plus_delta vector size");

    // Re-map member quaternion on input vector
    new (&quat1_) Eigen::Map<const Eigen::Quaternions>(_x.data()+3);

    _x_plus_delta.head(3) = _x.head(3) + quat1_*_delta.dp;
    _x_plus_delta.tail(4) = (quat1_*_delta.dq).coeffs();
}

// Compose two delta-frames and give a delta-frame
inline void ProcessorOdom3d::deltaPlusDelta(const Odom3dDelta& _delta1, const Odom3dDelta& _delta2, Odom3dDelta& _delta1_plus_delta2)
{
    _delta1_plus_delta2.dp = _delta1.dp + _delta1.dq*_delta2.dp;
    _delta1_plus_delta2.dq = _delta1.dq * _delta2.dq;
}

inline void ProcessorOdom3d::deltaMinusDelta(const Odom3dDelta& _delta1, const Odom3dDelta& _delta2,
                                             Odom3dDelta& _delta2_minus_delta1)
{
    _delta2_minus_delta1.dp = _delta1.dq.conjugate() * (_delta2.dp - _delta1.dp);
    _delta2_minus_delta1.dq = _delta1.dq.conjugate() * _delta2.dq;
}

inline Odom3dDelta ProcessorOdom3d::deltaZero() const
{
    //    return Odo3dDeltaType {Eigen::Vector3s::Zero(), Eigen::Quaternions::Identity()}; // explicit version
    //    return Odo3dDeltaType(); // Uses default constructor with null motion
    return Odom3dDelta::Zero(); //  Uses static member function
}

#endif /* SRC_PROCESSOR_ODOM_3D_H_ */
