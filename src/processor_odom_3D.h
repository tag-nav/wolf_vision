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
#include "capture_odom_3D.h"
#include "constraint_odom_3D.h"
#include "rotations.h"
#include <cmath>


namespace wolf {
    
WOLF_STRUCT_PTR_TYPEDEFS(ProcessorParamsOdom3D);

struct ProcessorParamsOdom3D : public ProcessorParamsBase
{
        Scalar max_time_span    = 1.0;
        Size   max_buff_length  = 10;
        Scalar dist_traveled    = 1.0;
        Scalar angle_turned     = 0.5;
};


WOLF_PTR_TYPEDEFS(ProcessorOdom3D);

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
        ProcessorOdom3D(const ProcessorParamsOdom3D& _params = ProcessorParamsOdom3D(), SensorOdom3DPtr _sensor_ptr = nullptr);
        virtual ~ProcessorOdom3D();
        virtual void configure(SensorBasePtr _sensor) override;

    public:
        virtual void computeCurrentDelta(const Eigen::VectorXs& _data,
                                         const Eigen::MatrixXs& _data_cov,
                                         const Eigen::VectorXs& _calib,
                                         const Scalar _dt,
                                         Eigen::VectorXs& _delta,
                                         Eigen::MatrixXs& _delta_cov,
                                         Eigen::MatrixXs& _jacobian_calib) override;
        void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                            const Eigen::VectorXs& _delta2,
                            const Scalar _Dt2,
                            Eigen::VectorXs& _delta1_plus_delta2) override;
        void deltaPlusDelta(const Eigen::VectorXs& _delta1,
                            const Eigen::VectorXs& _delta2,
                            const Scalar _Dt2,
                            Eigen::VectorXs& _delta1_plus_delta2,
                            Eigen::MatrixXs& _jacobian1,
                            Eigen::MatrixXs& _jacobian2) override;
        void statePlusDelta(const Eigen::VectorXs& _x,
                        const Eigen::VectorXs& _delta,
                        const Scalar _Dt,
                        Eigen::VectorXs& _x_plus_delta) override;
        Eigen::VectorXs deltaZero() const override;
        Motion interpolate(const Motion& _motion_ref,
                           Motion& _motion,
                           TimeStamp& _ts) override;
        bool voteForKeyFrame() override;
        virtual CaptureMotionPtr createCapture(const TimeStamp& _ts,
                                               const SensorBasePtr& _sensor,
                                               const VectorXs& _data,
                                               const MatrixXs& _data_cov,
                                               const FrameBasePtr& _frame_origin) override;
        virtual ConstraintBasePtr emplaceConstraint(FeatureBasePtr _feature_motion,
                                                    CaptureBasePtr _capture_origin) override;
        virtual FeatureBasePtr createFeature(CaptureMotionPtr _capture_motion) override;

    protected:
        // noise parameters (stolen from owner SensorOdom3D)
        Scalar k_disp_to_disp_; // displacement variance growth per meter of linear motion
        Scalar k_disp_to_rot_;  // orientation  variance growth per meter of linear motion
        Scalar k_rot_to_rot_;   // orientation  variance growth per radian of rotational motion
        Scalar min_disp_var_;   // floor displacement variance when no  motion
        Scalar min_rot_var_;    // floor orientation variance when no motion

        // keyframe voting parameters
        Scalar max_time_span_;  // maximum time between keyframes
        Size   max_buff_length_;// maximum buffer size before keyframe
        Scalar dist_traveled_;  // maximum linear motion between keyframes
        Scalar angle_turned_;   // maximum rotation between keyframes

        // Eigen::Map helpers
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

        void setAngleTurned(Scalar angleTurned)
        {
            angle_turned_ = angleTurned;
        }

        void setDistTraveled(Scalar distTraveled)
        {
            dist_traveled_ = distTraveled;
        }

        void setMaxBuffLength(Size maxBuffLength)
        {
            max_buff_length_ = maxBuffLength;
        }

        void setMaxTimeSpan(Scalar maxTimeSpan)
        {
            max_time_span_ = maxTimeSpan;
        }
};

inline Eigen::VectorXs ProcessorOdom3D::deltaZero() const
{
    return (Eigen::VectorXs(7) << 0,0,0, 0,0,0,1).finished(); // p, q
}

} // namespace wolf

#endif /* SRC_PROCESSOR_ODOM_3D_H_ */
