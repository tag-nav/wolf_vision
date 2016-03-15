/**
 * \file test_motion.cpp
 *
 *  Created on: Mar 15, 2016
 *      \author: jsola
 */


#include "processor_motion.h"

#include "state_block.h"
#include "state_quaternion.h"
#include "capture_motion.h"
#include "wolf.h"

class CaptureOdom3D : public CaptureMotion
{
    public:
        CaptureOdom3D(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr,
                      const Eigen::Vector7s& _data) :
                CaptureMotion(_init_ts, _final_ts, _sensor_ptr, _data)
        {
            //
        }
        virtual void integrateCapture(CaptureMotion* _new_capture){};
        virtual CaptureMotion* interpolateCapture(const TimeStamp& _ts){return this;}
        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now) const {return data_;}

};

class ProcessorOdom3d : public ProcessorMotion
{
    public:
        ProcessorOdom3d() : ProcessorMotion(PRC_ODOM_3D), d_angle_(0){}
        virtual ~ProcessorOdom3d(){}
        virtual void extractData(CaptureBase* _capture_ptr, TimeStamp& _ts, Eigen::VectorXs& _data);
        virtual void plus(const Eigen::VectorXs& _x, const Eigen::VectorXs& _dx, Eigen::VectorXs& _x_plus_dx);
        virtual void minus(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x0, Eigen::VectorXs& _x1_minus_x0);

    private:
        Eigen::Vector3s p0_, p1_, dp_;
        Eigen::Vector3s d_theta_, axis_;
        WolfScalar d_angle_;
        Eigen::Quaternions q0_, q1_, dq_;
};

inline void ProcessorOdom3d::extractData(CaptureBase* _capture_ptr, TimeStamp& _ts, Eigen::VectorXs& _data)
{
    _ts = _capture_ptr->getTimeStamp();
    _data = ((CaptureMotion*)(_capture_ptr))->getData(); // casting to derived Capture
}

inline void ProcessorOdom3d::plus(const Eigen::VectorXs& _x, const Eigen::VectorXs& _dx, Eigen::VectorXs& _x_plus_dx)
{
    assert(_x.size() == 7 && "Wrong _x vector size");
    assert(_x.size() == 6 && "Wrong _x vector size");
    assert(_x.size() == 7 && "Wrong _x vector size");
    p0_ = _x.head(3);
    q0_ = Eigen::Map<const Eigen::Quaternions>(&_x(3));
    dp_ = _dx.head(3);
    d_theta_ = dx_.tail(3);
    d_angle_ = d_theta_.norm();
    axis_ = d_theta_/d_angle_;
    dq_ = Eigen::Quaternions(Eigen::AngleAxiss(d_angle_,axis_));
    _x_plus_dx.head(3) = p0_ + q0_*dp_;
    _x_plus_dx.tail(4) = (q0_*dq_).coeffs();
}

inline void ProcessorOdom3d::minus(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x0,
                                   Eigen::VectorXs& _x1_minus_x0)
{
    // We accept two options for the output, depending on its size
    // size = 6: orientation returned in rotation vector angle*axis
    // size = 7: orientation in quaternion
    // In any case, we first compute the quaternion and then convert it to angle*axis if required

    p0_ = _x0.head(3);
    q0_ = Eigen::Map<const Eigen::Quaternions>(&_x0(3));
    p1_ = _x0.head(3);
    q1_ = Eigen::Map<const Eigen::Quaternions>(&_x1(3));

    dp_ = q0_.conjugate()*(p1_-p0_);
    dq_ = q0_.conjugate()*q1_;

    _x1_minus_x0.head(3) = dp_;
    switch (_x1_minus_x0.size())
    {
        case 6: // make rotation vector via angle/axis
        {
            Eigen::AngleAxiss v(dq_);
            _x1_minus_x0.tail(4) = v.angle() * v.axis();
            break;
        }
        case 7: // assign quaternion coefficients
        {
            _x1_minus_x0.tail(4) = dq_.coeffs();
            break;
        }
    }

}


#include <iostream>

int main()
{
    // time
    TimeStamp t;
    t.setToNow();

    // robot state blocks: origin
    StateBlock sb_pos(Eigen::Vector3s::Zero());
    StateQuaternion sb_ori(Eigen::Quaternions::Identity().coeffs());

    // sensor state blocks: the same as robot
    StateBlock sb_intr(Eigen::VectorXs::Zero(0));

    // create sensor
    SensorBase sensor(ODOM_2D, &sb_pos, &sb_ori, &sb_intr, 0);

    // motion data
    Eigen::VectorXs data(7);
    data << 1, 2, 3, 4, 5, 6, 7;
    data.tail(4).normalize();

    std::cout << "Data: " << data.transpose() << std::endl;

    FrameBase* frm_ptr = new FrameBase(t, &sb_pos, &sb_ori);
    CaptureMotion* cap_ptr = new CaptureOdom3D(t-1, t, &sensor, data);
    frm_ptr->addCapture(cap_ptr);

    // Make a ProcessorOdom3d
    ProcessorOdom3d odom3d;
    odom3d.init(cap_ptr);

    // Add a capture to process
    odom3d.process(cap_ptr);


    return 0;
}
