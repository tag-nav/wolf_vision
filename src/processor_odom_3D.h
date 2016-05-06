/**
 * \file processor_odom_3D.h
 *
 *  Created on: Mar 18, 2016
 *      \author: jsola
 */

#ifndef SRC_PROCESSOR_ODOM_3D_H_
#define SRC_PROCESSOR_ODOM_3D_H_

#include "processor_motion.h"
#include "constraint_odom_2D.h"


namespace wolf {

/** \brief Processor for 3d odometry integration.
 *
 * This processor integrates motion data in the form of 3D odometry.
 *
 * The odometry data is extracted from Captures of the type CaptureOdometry3d.
 * This data comes in the form of a 6-vector, containing the following components:
 *   - a 3d position increment in the local frame of the robot (dx, dy, dz)
 *   - a 3d orientation increment in the local frame of the robot (roll, pitch, yaw)
 *
 * The produced integrated deltas are in the form of 7-vectors with the following components:
 *   - a 3d position increment in the local frame of the robot (dx, dy, dz)
 *   - a quaternion orientation increment in the local frame of the robot (qx, qy, qz, qw)
 *
 * The produced states are in the form of 7-vectors with the following components:
 *   - a 3d position increment in the local frame of the robot (dx, dy, dz)
 *   - a quaternion orientation increment in the local frame of the robot (qx, qy, qz, qw)
 *
 * The processor integrates data by ignoring the time increment dt_
 * (as it integrates motion directly, not velocities).
 *
 * All frames are assumed FLU (front, left, up).
 */
class ProcessorOdom3D : public ProcessorMotion
{
    public:
        ProcessorOdom3D();
        virtual ~ProcessorOdom3D();
        virtual void data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt,
                                Eigen::VectorXs& _delta, Eigen::MatrixXs& _delta_cov);

    protected:
//        virtual void preProcess(){}
//        virtual void postProcess(){}

    private:
        void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta);
        void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2);
        void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                            Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                            Eigen::MatrixXs& _jacobian2);
        virtual void deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                     Eigen::VectorXs& _delta2_minus_delta1);
        Eigen::VectorXs deltaZero() const;
        Motion interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts);

        virtual ConstraintBase* createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin);

    private:
        Eigen::Map<const Eigen::Vector3s> p1_, p2_;
        Eigen::Map<Eigen::Vector3s> p_out_;
        Eigen::Map<const Eigen::Quaternions> q1_, q2_;
        Eigen::Map<Eigen::Quaternions> q_out_;
        void remap(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x2, Eigen::VectorXs& _x_out);

    // Factory method
    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};


inline ProcessorOdom3D::ProcessorOdom3D() :
        ProcessorMotion(PRC_ODOM_3D, 7, 7, 6),
        p1_(nullptr),
        p2_(nullptr),
        p_out_(nullptr),
        q1_(nullptr),
        q2_(nullptr),
        q_out_(nullptr)
{
    setType("ODOM 3D");
}

inline ProcessorOdom3D::~ProcessorOdom3D()
{
}

inline void ProcessorOdom3D::data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt,
                                        Eigen::VectorXs& _delta, Eigen::MatrixXs& _delta_cov)
{
    _delta.head(3) = _data.head(3);
    new (&q_out_) Eigen::Map<Eigen::Quaternions>(_delta.data() + 3);

    Eigen::v2q(_data.tail(3), q_out_);
    // TODO: fill delta covariance
    _delta_cov = Eigen::MatrixXs::Identity(delta_size_, delta_size_) * 0.01;
}

inline void ProcessorOdom3D::xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta)
{
    assert(_x.size() == 7 && "Wrong _x vector size");
    assert(_delta.size() == 7 && "Wrong _delta vector size");
    assert(_x_plus_delta.size() == 7 && "Wrong _x_plus_delta vector size");

    remap(_x, _delta, _x_plus_delta);

    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;
}

inline void ProcessorOdom3D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2)
{
    assert(_delta1.size() == 7 && "Wrong _delta1 vector size");
    assert(_delta2.size() == 7 && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == 7 && "Wrong _delta1_plus_delta2 vector size");

    remap(_delta1, _delta2, _delta1_plus_delta2);
    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;
}

inline void ProcessorOdom3D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                            Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                            Eigen::MatrixXs& _jacobian2)
{
    assert(_delta1.size() == 7 && "Wrong _delta1 vector size");
    assert(_delta2.size() == 7 && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == 7 && "Wrong _delta1_plus_delta2 vector size");
    // TODO: assert sizes of jacobians

    remap(_delta1, _delta2, _delta1_plus_delta2);
    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;

    // TODO: fill the jacobians
    _jacobian1 = Eigen::MatrixXs::Identity(delta_size_,delta_size_);
    _jacobian2 = Eigen::MatrixXs::Identity(delta_size_,delta_size_);
}

inline void ProcessorOdom3D::deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                             Eigen::VectorXs& _delta2_minus_delta1)
{
    assert(_delta1.size() == 7 && "Wrong _delta1 vector size");
    assert(_delta2.size() == 7 && "Wrong _delta2 vector size");
    assert(_delta2_minus_delta1.size() == 7 && "Wrong _delta2_minus_delta1 vector size");

    remap(_delta1, _delta2, _delta2_minus_delta1);
    p_out_ = q1_.conjugate() * (p2_ - p1_);
    q_out_ = q1_.conjugate() * q2_;
}

inline Eigen::VectorXs ProcessorOdom3D::deltaZero() const
{
    Eigen::VectorXs delta_zero(7);
    delta_zero << 0, 0, 0, 0, 0, 0, 1;;
    return delta_zero;
}

inline Motion ProcessorOdom3D::interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts)
{
    Motion tmp(_motion_ref);
    tmp.ts_ = _ts;
    tmp.delta_ = deltaZero();
    tmp.delta_cov_ = Eigen::MatrixXs::Zero(delta_size_, delta_size_);
    return tmp;
}

inline ConstraintBase* ProcessorOdom3D::createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin)
{
    return new ConstraintOdom2D(_feature_motion, _frame_origin);
}

inline void ProcessorOdom3D::remap(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x2, Eigen::VectorXs& _x_out)
{
    new (&p1_) Eigen::Map<const Eigen::Vector3s>(_x1.data());
    new (&q1_) Eigen::Map<const Eigen::Quaternions>(_x1.data() + 3);
    new (&p2_) Eigen::Map<const Eigen::Vector3s>(_x2.data());
    new (&q2_) Eigen::Map<const Eigen::Quaternions>(_x2.data() + 3);
    new (&p_out_) Eigen::Map<Eigen::Vector3s>(_x_out.data());
    new (&q_out_) Eigen::Map<Eigen::Quaternions>(_x_out.data() + 3);
}

ProcessorBase* ProcessorOdom3D::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorOdom3D* prc_ptr = new ProcessorOdom3D();
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}


} // namespace wolf



// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
//const bool registered_prc_odom_3d = ProcessorFactory::get()->registerCreator("ODOM 3D", ProcessorOdom3D::create);
}
} // namespace wolf


#endif /* SRC_PROCESSOR_ODOM_3D_H_ */
