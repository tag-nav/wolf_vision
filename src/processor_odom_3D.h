/**
 * \file processor_odom_3D.h
 *
 *  Created on: Mar 18, 2016
 *      \author: jsola
 */

#ifndef SRC_PROCESSOR_ODOM_3D_H_
#define SRC_PROCESSOR_ODOM_3D_H_

#include "processor_motion.h"
#include "sensor_odom_3D.h"
#include "constraint_odom_3D.h"
#include "rotations.h"


namespace wolf {

/** \brief Processor for 3d odometry integration.
 *
 * This processor integrates motion data in the form of 3D odometry.
 *
 * The odometry data is extracted from Captures of the type CaptureOdometry3d.
 * This data comes in the form of a 6-vector, or a 7-vector, containing the following components:
 *   - a 3d position increment in the local frame of the robot (dx, dy, dz).
 *   - a 3d orientation increment in the local frame of the robot (droll, dpitch, dyaw), or quaternion (dqx, dqy, dqz, dqw).
 *
 * The produced integrated deltas are in the form of 7-vectors with the following components:
 *   - a 3d position increment in the local frame of the robot (Dx, Dy, Dz)
 *   - a quaternion orientation increment in the local frame of the robot (Dqx, Dqy, Dqz, Dqw)
 *
 * The produced states are in the form of 7-vectors with the following components:
 *   - a 3d position of the robot in the world frame (x, y, z)
 *   - a quaternion orientation of the robot in the world frame (qx, qy, qz, qw)
 *
 * The processor integrates data by ignoring the time increment dt_
 * (as it integrates motion directly, not velocities).
 *
 * All frames are assumed FLU ( x: Front, y: Left, z: Up ).
 */
class ProcessorOdom3D : public ProcessorMotion
{
    public:
        typedef std::shared_ptr<ProcessorOdom3D> Ptr;

    public:
        ProcessorOdom3D(Scalar _k_disp_to_disp = 0.1,
                        Scalar _k_disp_to_rot = 0.1,
                        Scalar _k_rot_to_rot = 0.1,
                        Scalar _min_disp_var = 0.1,
                        Scalar _min_rot_var = 0.1);
        virtual ~ProcessorOdom3D();
        void setup(SensorOdom3D::Ptr sen_ptr);

    public:
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
        bool voteForKeyFrame();
        ConstraintBasePtr createConstraint(FeatureBasePtr _feature_motion,
                                           FrameBasePtr _frame_origin);

    private:
        Scalar k_disp_to_disp_, k_disp_to_rot_, k_rot_to_rot_;
        Scalar min_disp_var_, min_rot_var_;
        Eigen::Map<const Eigen::Vector3s> p1_, p2_;
        Eigen::Map<Eigen::Vector3s> p_out_;
        Eigen::Map<const Eigen::Quaternions> q1_, q2_;
        Eigen::Map<Eigen::Quaternions> q_out_;
        void remap(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x2, Eigen::VectorXs& _x_out);

    // Factory method
    public:
        static ProcessorBasePtr create(const std::string& _unique_name,
                                       const ProcessorParamsBasePtr _params,
                                       const SensorBasePtr sensor_ptr = nullptr);
};

inline Eigen::VectorXs ProcessorOdom3D::deltaZero() const
{
    return (Eigen::VectorXs(7) << 0,0,0, 0,0,0,1).finished(); // p, q
}

inline bool ProcessorOdom3D::voteForKeyFrame()
{
    if (getBuffer().get().size() > 5)
    {
        std::cout << "PM::vote buffer big enough" << std::endl;
        return true;
    }
    if (delta_integrated_.head(3).norm() > 1)
    {
        std::cout << "PM::vote position delta big enough" << std::endl;
        return true;
    }
    if (delta_integrated_.tail(3).norm() > 0.5)
    {
        std::cout << "PM::vote orientation delta big enough" << std::endl;
        return true;
    }

    std::cout << "PM::do not vote" << std::endl;
    return false;
}

inline ConstraintBasePtr ProcessorOdom3D::createConstraint(FeatureBasePtr _feature_motion,
                                                           FrameBasePtr _frame_origin)
{
    ConstraintOdom3D::Ptr ctr_odom = std::make_shared<ConstraintOdom3D>(_feature_motion, _frame_origin);
    return ctr_odom;
}

inline void ProcessorOdom3D::remap(const Eigen::VectorXs& _x1,
                                   const Eigen::VectorXs& _x2,
                                   Eigen::VectorXs& _x_out)
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
