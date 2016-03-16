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
                      const Eigen::Vector6s& _data) :
                CaptureMotion(_init_ts, _final_ts, _sensor_ptr, _data)
        {
            //
        }

        // INFO: These functions are useless but pure virtual so I define them with dummy code. I am not using them.
        virtual void integrateCapture(CaptureMotion* _new_capture){};
        virtual CaptureMotion* interpolateCapture(const TimeStamp& _ts){return this;}
        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now) const {return data_;}

};

// This is the class under test!
class ProcessorOdom3d : public ProcessorMotion
{
    public:
        ProcessorOdom3d() : ProcessorMotion(PRC_ODOM_3D, 7, 6, 6), d_angle_(0){}
        virtual ~ProcessorOdom3d(){}
        virtual void extractData(CaptureBase* _capture_ptr, TimeStamp& _ts, Eigen::VectorXs& _data);
        virtual void plus(const Eigen::VectorXs& _x, const Eigen::VectorXs& _dx, Eigen::VectorXs& _x_plus_dx);
        virtual void minus(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x0, Eigen::VectorXs& _x1_minus_x0);

    private:
        void plus777(const Eigen::VectorXs& _x, const Eigen::VectorXs& _dx, Eigen::VectorXs& _x_plus_dx);
        void plus767(const Eigen::VectorXs& _x0, const Eigen::VectorXs& _x1, Eigen::VectorXs& _x0_plus_x1);
        void plus666(const Eigen::VectorXs& _x, const Eigen::VectorXs& _dx, Eigen::VectorXs& _x_plus_dx);

    private:
        Eigen::Vector3s p0_, p1_, dp_;
        Eigen::Vector3s d_theta_, axis_;
        WolfScalar d_angle_;
        Eigen::Quaternions q0_, q1_, dq_;
};

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
    if (aa.angle() < WolfConstants::EPS)
        _v.setZero();
    else
        _v = aa.axis()/aa.angle();
}

inline void ProcessorOdom3d::extractData(CaptureBase* _capture_ptr, TimeStamp& _ts, Eigen::VectorXs& _data)
{
    _ts = _capture_ptr->getTimeStamp();
    _data = ((CaptureMotion*)(_capture_ptr))->getData(); // casting to derived Capture
}

void ProcessorOdom3d::plus(const Eigen::VectorXs& _x, const Eigen::VectorXs& _dx, Eigen::VectorXs& _x_plus_dx)
{
    std::cout << "plus: Initial pose  : " << _x.transpose() << std::endl;
    std::cout << "plus: Delta pose  : " << _dx.transpose() << std::endl;

    switch (_x.size())
    {
        case 6:
        {
            switch (_dx.size())
            {
                case 6:
                    plus666(_x, _dx, _x_plus_dx);
                    break;
                default:
                    assert(0 && "Wrong input vector sizes.");
                    break;
            }
        }
            break;
        case 7:
        {
            switch (_dx.size())
            {
                case 6:
                    plus767(_x, _dx, _x_plus_dx);
                    break;
                case 7:
                    plus777(_x, _dx, _x_plus_dx);
                    break;
                default:
                    assert(0 && "Wrong input vector sizes.");
            }
        }
            break;
        default:
            assert(0 && "Wrong input vector sizes.");
    }

    std::cout << "plus: Final pose  : " << _x_plus_dx.transpose() << std::endl;

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

// INFO: Frame: position + quaternion, size 7.
// INFO: delta-frame: position + rotation vector, size 6.

// Compose 2 frames and give a frame (all are size 7)
inline void ProcessorOdom3d::plus777(const Eigen::VectorXs& _x, const Eigen::VectorXs& _dx, Eigen::VectorXs& _x_plus_dx)
{
    assert(_x.size() == 7 && "Wrong _x vector size");
    assert(_dx.size() == 7 && "Wrong _dx vector size");
    assert(_x_plus_dx.size() == 7 && "Wrong _x_plus_dx vector size");

    p0_ = _x.head(3);
    q0_ = Eigen::Map<const Eigen::Quaternions>(&_x(3));
    dp_ = _dx.head(3);
    dq_ = Eigen::Quaternions(_dx.tail(4).data());
    _x_plus_dx.head(3) = p0_ + q0_*dp_;
    _x_plus_dx.tail(4) = (q0_*dq_).coeffs();
}

// Compose a frame with a delta frame and give a frame (sizes 7, 6, 7 respectively)
inline void ProcessorOdom3d::plus767(const Eigen::VectorXs& _x0, const Eigen::VectorXs& _x1, Eigen::VectorXs& _x0_plus_x1)
{
    assert(_x0.size() == 7 && "Wrong _x vector size");
    assert(_x1.size() == 6 && "Wrong _dx vector size");
    assert(_x0_plus_x1.size() == 7 && "Wrong _x_plus_dx vector size");

    p0_ = _x0.head(3);
    q0_ = Eigen::Map<const Eigen::Quaternions>(&_x0(3));
    dp_ = _x1.head(3);
    v2q(_x1.tail(3),dq_);
    _x0_plus_x1.head(3) = p0_ + q0_*dp_;
    _x0_plus_x1.tail(4) = (q0_*dq_).coeffs();
}

// Compose two delta-frames and give a delta-frame (all of size 6)
inline void ProcessorOdom3d::plus666(const Eigen::VectorXs& _x, const Eigen::VectorXs& _dx, Eigen::VectorXs& _x_plus_dx)
{
    assert(_x.size() == 6 && "Wrong _x vector size");
    assert(_dx.size() == 6 && "Wrong _dx vector size");
    assert(_x_plus_dx.size() == 6 && "Wrong _x_plus_dx vector size");

    p0_ = _x.head(3);
    q0_ = Eigen::Map<const Eigen::Quaternions>(&_x(3));
    dp_ = _dx.head(3);
    v2q(_dx.tail(3),dq_);

    Eigen::VectorXs v(3);
    q2v(q0_*dq_,v);
    _x_plus_dx.head(3) = p0_ + q0_*dp_;
    _x_plus_dx.tail(3) = v;
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
    Eigen::VectorXs data(6);
    data << 1, 2, 3, 4, 5, 6;


    std::cout << "Initial pose : " << sb_pos.getVector().transpose() << " " << sb_ori.getVector().transpose() << std::endl;
    std::cout << "Motion data  : " << data.transpose() << std::endl;

    FrameBase* frm_ptr = new FrameBase(t, &sb_pos, &sb_ori);
    CaptureMotion* cap_ptr = new CaptureOdom3D(t-1, t, &sensor, data);
    frm_ptr->addCapture(cap_ptr);

    // Make a ProcessorOdom3d
    ProcessorOdom3d odom3d;
    odom3d.init(cap_ptr);
    std::cout << "Initial pose : " << odom3d.x_origin_.transpose() << " " << sb_ori.getVector().transpose() << std::endl;
    std::cout << "Motion data  : " << odom3d.data_.transpose() << std::endl;
    std::cout << "Motion delta : " << odom3d.dx_.transpose() << std::endl;
    std::cout << "Delta integr : " << odom3d.Dx_integral_.transpose() << std::endl;

    // Add a capture to process
    odom3d.process(cap_ptr);

    std::cout << "Initial pose : " << odom3d.x_origin_.transpose() << " " << sb_ori.getVector().transpose() << std::endl;
    std::cout << "Motion data  : " << odom3d.data_.transpose() << std::endl;
    std::cout << "Motion delta : " << odom3d.dx_.transpose() << std::endl;
    std::cout << "Delta integr : " << odom3d.Dx_integral_.transpose() << std::endl;


    return 0;
}
