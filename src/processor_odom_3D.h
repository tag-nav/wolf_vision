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
#include "rotations.h"


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

    private:
        virtual void data2delta(const Eigen::VectorXs& _data,
                                const Eigen::MatrixXs& _data_cov,
                                const Scalar _dt);
        void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                            const Eigen::VectorXs& _delta2,
                            const Scalar _Dt2,
                            Eigen::VectorXs& _delta1_plus_delta2);
        void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                            const Eigen::VectorXs& _delta2,
                            const Scalar _Dt2,
                            Eigen::VectorXs& _delta1_plus_delta2,
                            Eigen::MatrixXs& _jacobian1,
                            Eigen::MatrixXs& _jacobian2);
        void xPlusDelta(const Eigen::VectorXs& _x,
                        const Eigen::VectorXs& _delta,
                        const Scalar _Dt,
                        Eigen::VectorXs& _x_plus_delta);
        Eigen::VectorXs deltaZero() const;
        Motion interpolate(const Motion& _motion_ref,
                           Motion& _motion,
                           TimeStamp& _ts);

        virtual ConstraintBasePtr createConstraint(FeatureBasePtr _feature_motion, FrameBasePtr _frame_origin);

    private:
        Eigen::Map<const Eigen::Vector3s> p1_, p2_;
        Eigen::Map<Eigen::Vector3s> p_out_;
        Eigen::Map<const Eigen::Quaternions> q1_, q2_;
        Eigen::Map<Eigen::Quaternions> q_out_;
        void remap(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x2, Eigen::VectorXs& _x_out);

    // Factory method
    public:
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params);
};

inline void ProcessorOdom3D::data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt)
{
    delta_.head(3) = _data.head(3);
    new (&q_out_) Eigen::Map<Eigen::Quaternions>(delta_.data() + 3);
    q_out_ = v2q(_data.tail<3>());

    /* Jacobians of d = data2delta(data, dt)
     * with: d =    [Dp Dq]
     *       data = [dp do]
     *
     *       Dp = dp
     *       Dq = v2q(do)
     *
     * dDp/ddp = I
     * dDp/ddo = 0
     * dDo/ddp = 0
     * dDo/ddo = I
     *
     * so, J = I, and delta_cov = _data_cov
     */
    delta_cov_ = _data_cov;
}

inline void ProcessorOdom3D::xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _Dt, Eigen::VectorXs& _x_plus_delta)
{
    assert(_x.size() == x_size_ && "Wrong _x vector size");
    assert(_delta.size() == delta_size_ && "Wrong _delta vector size");
    assert(_x_plus_delta.size() == x_size_ && "Wrong _x_plus_delta vector size");

    remap(_x, _delta, _x_plus_delta);

    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;

}

inline void ProcessorOdom3D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, const Scalar _Dt2, Eigen::VectorXs& _delta1_plus_delta2)
{
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");

    remap(_delta1, _delta2, _delta1_plus_delta2);
    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;
}

inline void ProcessorOdom3D::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                            const Scalar _Dt2,
                                            Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                            Eigen::MatrixXs& _jacobian2)
{
    assert(_delta1.size() == delta_size_ && "Wrong _delta1 vector size");
    assert(_delta2.size() == delta_size_ && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == delta_size_ && "Wrong _delta1_plus_delta2 vector size");
    assert(_jacobian1.rows() == delta_cov_size_ && _jacobian1.cols() == delta_cov_size_ && "Wrong _jacobian1 size");
    assert(_jacobian2.rows() == delta_cov_size_ && _jacobian2.cols() == delta_cov_size_ && "Wrong _jacobian2 size");

    remap(_delta1, _delta2, _delta1_plus_delta2);
    p_out_ = p1_ + q1_ * p2_;
    q_out_ = q1_ * q2_;

    /* Jacobians of D' = D (+) d
     * with: D = [Dp Dq]
     *       d = [dp dq]
     *
     * dDp'/dDp = I
     * dDp'/dDo = -DR * skew(dp)   // (Sola 16, ex. B.3.2 and Sec. 7.2.3)
     * dDo'/dDp = 0
     * dDo'/dDo = dR.tr            // (Sola 16, Sec. 7.2.3)
     *
     * dDp'/ddp = DR
     * dDp'/ddo = 0
     * dDo'/ddp = 0
     * dDo'/ddo = I
     */

    // temporaries
    Eigen::Matrix3s DR = q1_.matrix();
    Eigen::Matrix3s dR = q2_.matrix();

    // fill Jacobians -- parts not shown are constant and set at construction time.
    _jacobian1.block<3,3>(0,3) = - DR * skew(p2_);  // (Sola 16, ex. B.3.2 and Sec. 7.2.3)
    _jacobian1.block<3,3>(3,3) = dR.transpose();    // (Sola 16, Sec. 7.2.3)
    _jacobian2.block<3,3>(0,0) = DR;                // (Sola 16, Sec. 7.2.3)
}

inline Eigen::VectorXs ProcessorOdom3D::deltaZero() const
{
    return (Eigen::VectorXs(7) << 0,0,0, 0,0,0,1).finished(); // p, q
}

inline Motion ProcessorOdom3D::interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts)
{
    Motion tmp(_motion_ref);
    tmp.ts_ = _ts;
    tmp.delta_ = deltaZero();
    tmp.delta_cov_ = Eigen::MatrixXs::Zero(delta_cov_size_, delta_cov_size_);
    return tmp;
}

inline ConstraintBasePtr ProcessorOdom3D::createConstraint(FeatureBasePtr _feature_motion, FrameBasePtr _frame_origin)
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

} // namespace wolf

#endif /* SRC_PROCESSOR_ODOM_3D_H_ */
