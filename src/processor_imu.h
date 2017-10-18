#ifndef PROCESSOR_IMU_H
#define PROCESSOR_IMU_H

// Wolf
#include "processor_motion.h"
#include "frame_imu.h"
#include "capture_imu.h"
#include "feature_imu.h"


namespace wolf {
WOLF_STRUCT_PTR_TYPEDEFS(ProcessorIMUParams);

struct ProcessorIMUParams : public ProcessorParamsBase
{
        Scalar max_time_span;
        Size   max_buff_length;
        Scalar dist_traveled;
        Scalar angle_turned;
        bool voting_active; //IMU will not vote for key Frames to be created


        ProcessorIMUParams() :
            max_time_span(0.5),
            max_buff_length(10),
            dist_traveled(5),
            angle_turned(.5),
            voting_active(false)
        {
            type = "IMU";
            name = "";
        }
};

WOLF_PTR_TYPEDEFS(ProcessorIMU);
    
//class
class ProcessorIMU : public ProcessorMotion{
    public:
        ProcessorIMU(ProcessorIMUParamsPtr _params = nullptr);
        virtual ~ProcessorIMU();

    protected:
        virtual void computeCurrentDelta(const Eigen::VectorXs& _data,
                                         const Eigen::MatrixXs& _data_cov,
                                         const Eigen::VectorXs& _calib,
                                         const Scalar _dt,
                                         Eigen::VectorXs& _delta,
                                         Eigen::MatrixXs& _delta_cov,
                                         Eigen::MatrixXs& _jacobian_calib) override;
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta_preint,
                                    const Eigen::VectorXs& _delta,
                                    const Scalar _dt,
                                    Eigen::VectorXs& _delta_preint_plus_delta) override;
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta_preint,
                                    const Eigen::VectorXs& _delta,
                                    const Scalar _dt,
                                    Eigen::VectorXs& _delta_preint_plus_delta,
                                    Eigen::MatrixXs& _jacobian_delta_preint,
                                    Eigen::MatrixXs& _jacobian_delta) override;
        virtual void statePlusDelta(const Eigen::VectorXs& _x,
                                const Eigen::VectorXs& _delta,
                                const Scalar _dt,
                                Eigen::VectorXs& _x_plus_delta ) override;
        virtual Eigen::VectorXs deltaZero() const override;
        virtual Motion interpolate(const Motion& _motion_ref,
                                   Motion& _motion,
                                   TimeStamp& _ts) override;
        virtual bool voteForKeyFrame() override;
        virtual CaptureMotionPtr emplaceCapture(const TimeStamp& _ts,
                                                const SensorBasePtr& _sensor,
                                                const VectorXs& _data,
                                                const MatrixXs& _data_cov,
                                                const FrameBasePtr& _frame_own,
                                                const FrameBasePtr& _frame_origin) override;
        virtual FeatureBasePtr emplaceFeature(CaptureMotionPtr _capture_motion,
                                              FrameBasePtr _related_frame) override;
        virtual ConstraintBasePtr emplaceConstraint(FeatureBasePtr _feature_motion,
                                                    FrameBasePtr _frame_origin) override;
        void resetDerived() override;

    protected:

        // keyframe voting parameters
        Scalar max_time_span_;  // maximum time between keyframes
        Size   max_buff_length_;// maximum buffer size before keyframe
        Scalar dist_traveled_;  // maximum linear motion between keyframes
        Scalar angle_turned_;   // maximum rotation between keyframes
        bool voting_active_;    // IMU will be voting for KeyFrame only if this is true

        // Casted pointer to IMU frame
        FrameIMUPtr frame_imu_ptr_;

        // gravity vector
        const Eigen::Vector3s gravity_;

        // Biases in the first keyframe's state for pre-integration
        Eigen::Map<Eigen::Vector3s> acc_bias_;
        Eigen::Map<Eigen::Vector3s> gyro_bias_;

        // Maps to the received measurements
        Eigen::Map<Eigen::Vector3s> acc_measured_;
        Eigen::Map<Eigen::Vector3s> gyro_measured_;

        // Maps to pos, vel, quat, to be used as temporaries
        Eigen::Map<const Eigen::Vector3s> Dp_, dp_;
        Eigen::Map<Eigen::Vector3s> Dp_out_;
        Eigen::Map<const Eigen::Vector3s> Dv_, dv_;
        Eigen::Map<Eigen::Vector3s> Dv_out_;
        Eigen::Map<const Eigen::Quaternions> Dq_, dq_;
        Eigen::Map<Eigen::Quaternions> Dq_out_;

        // Helper functions to remap several magnitudes
//        virtual void remapDelta(Eigen::VectorXs& _delta_out);

    public:
        //getters
        Scalar getMaxTimeSpan() const;
        Scalar getMaxBuffLength() const;
        Scalar getDistTraveled() const;
        Scalar getAngleTurned() const;
        //for factory
        static ProcessorBasePtr create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr sensor_ptr = nullptr);
};

}

/////////////////////////////////////////////////////////
// IMPLEMENTATION. Put your implementation includes here
/////////////////////////////////////////////////////////

// Wolf
#include "constraint_imu.h"
#include "state_block.h"
#include "rotations.h"


namespace wolf{

inline void ProcessorIMU::computeCurrentDelta(const Eigen::VectorXs& _data,
                                              const Eigen::MatrixXs& _data_cov,
                                              const Eigen::VectorXs& _calib,
                                              const Scalar _dt,
                                              Eigen::VectorXs& _delta,
                                              Eigen::MatrixXs& _delta_cov,
                                              Eigen::MatrixXs& _jacobian_calib)
{
    assert(_data.size() == data_size_ && "Wrong data size!");

    using namespace Eigen;

    // acc and gyro measurements corrected with the estimated bias, times dt
    Vector3s a_dt = (_data.head(3) - _calib.head(3)) * _dt;
    Vector3s w_dt = (_data.tail(3) - _calib.tail(3)) * _dt;

    /* create delta
     *
     * MATHS of delta creation -- Sola-16
     * dp = 1/2 * (a-a_b) * dt^2 = 1/2 * dv * dt
     * dv = (a-a_b) * dt
     * dq = exp((w-w_b)*dt)
     */
    Vector3s delta_p = a_dt * _dt / 2;
    Quaternions delta_q = v2q(w_dt);
    Vector3s delta_v = a_dt;
    _delta << delta_p , delta_q.coeffs() , delta_v;


    /* Compute jacobian of delta wrt data
     *
     * MATHS : jacobian dd_dn, of delta wrt noise
     * substituting a and w respectively by (a+a_n) and (w+w_n) (measurement noise is additive)
     *                 an           wn
     *         dp [ 0.5*I*dt*dt     0     ]
     * dd_dn = do [    0           Jr*dt  ]
     *         dv [    I*dt         0     ] // see Sola-16
     */

    // we go the sparse way:
    Matrix3s ddv_dan = Matrix3s::Identity() * _dt;
    Matrix3s ddp_dan = ddv_dan * _dt / 2;
    Matrix3s ddo_dwn = jac_SO3_right(w_dt) * _dt;

    /* Covariance computation
     *
     * Covariance is sparse:
     *       [ Cpp   0   Cpv
     * COV =    0   Coo   0
     *         Cpv'  0   Cvv ]
     *
     * where Cpp, Cpv, Coo and Cvv are computed below
     */
    _delta_cov.block<3,3>(0,0).noalias() = ddp_dan*_data_cov.block<3,3>(0,0)*ddp_dan.transpose(); // Cpp = ddp_dan * Caa * ddp_dan'
    _delta_cov.block<3,3>(0,6).noalias() = ddp_dan*_data_cov.block<3,3>(0,0)*ddv_dan.transpose(); // Cpv = ddp_dan * Caa * ddv_dan'
    _delta_cov.block<3,3>(3,3).noalias() = ddo_dwn*_data_cov.block<3,3>(3,3)*ddo_dwn.transpose(); // Coo = ddo_dwn * Cww * ddo_dwn'
    _delta_cov.block<3,3>(6,0)           = _delta_cov.block<3,3>(0,6).transpose();                // Cvp = Cpv'
    _delta_cov.block<3,3>(6,6).noalias() = ddv_dan*_data_cov.block<3,3>(0,0)*ddv_dan.transpose(); // Cvv = ddv_dan * Caa * ddv_dan'


    /* Jacobians of delta wrt calibration parameters -- bias
     *
     * We know that d_(meas - bias)/d_bias = -I
     * so d_delta/d_bias = - d_delta/d_meas
     * we assign only the non-null ones
     */
    _jacobian_calib.setZero(delta_cov_size_,calib_size_); // can be commented usually, more sure this way
    _jacobian_calib.block(0,0,3,3) = - ddp_dan;
    _jacobian_calib.block(3,3,3,3) = - ddo_dwn;
    _jacobian_calib.block(6,0,3,3) = - ddv_dan;

}

inline void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta_preint,
                                         const Eigen::VectorXs& _delta,
                                         const Scalar _dt,
                                         Eigen::VectorXs& _delta_preint_plus_delta,
                                         Eigen::MatrixXs& _jacobian_delta_preint,
                                         Eigen::MatrixXs& _jacobian_delta)
{

    /*
     * Expression of the delta integration step, D' = D (+) d:
     *
     *     Dp' = Dp + Dv*dt + Dq*dp
     *     Dv' = Dv + Dq*dv
     *     Dq' = Dq * dq
     */

    /*/////////////////////////////////////////////////////////
     * Note. Jacobians for covariance propagation.
     *
     * 1.a. With respect to Delta, gives _jacobian_delta_preint = D_D as:
     *
     *   D_D = [ I    -DR*skew(dp)   I*dt
     *           0     dR.tr          0
     *           0    -DR*skew(dv)    I  ] // See Sola-16
     *
     * 1.b. With respect to delta, gives _jacobian_delta = D_d as:
     *
     *   D_d = [ DR   0    0
     *           0    I    0
     *           0    0    DR ] // See Sola-16
     *
     * Note: covariance propagation, i.e.,  P+ = D_D * P * D_D' + D_d * M * D_d', is done in ProcessorMotion.
     */

    imu::compose(_delta_preint, _delta, _dt, _delta_preint_plus_delta, _jacobian_delta_preint, _jacobian_delta);

}

inline void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta_preint,
                                         const Eigen::VectorXs& _delta,
                                         const Scalar _dt,
                                         Eigen::VectorXs& _delta_preint_plus_delta)
{

    /* MATHS according to Sola-16
     * Dp' = Dp + Dv*dt + 1/2*Dq*(a-a_b)*dt^2    = Dp + Dv*dt + Dq*dp   if  dp = 1/2*(a-a_b)*dt^2
     * Dv' = Dv + Dq*(a-a_b)*dt                  = Dv + Dq*dv           if  dv = (a-a_b)*dt
     * Dq' = Dq * exp((w-w_b)*dt)                = Dq * dq              if  dq = exp((w-w_b)*dt)
     *
     * where (dp, dq, dv) need to be computed in data2delta(), and Dq*dx =is_equivalent_to= Dq*dx*Dq'.
     */

    imu::compose(_delta_preint, _delta, _delta_preint_plus_delta);

}

inline void ProcessorIMU::statePlusDelta(const Eigen::VectorXs& _x,
                                     const Eigen::VectorXs& _delta,
                                     const Scalar _dt,
                                     Eigen::VectorXs& _x_plus_delta)
{
//    assert(_x.size() == 10 && "Wrong _x vector size");
//    assert(_delta.size() == 10 && "Wrong _delta vector size");
//    assert(_x_plus_delta.size() == 10 && "Wrong _x_plus_delta vector size");
    assert(_dt >= 0 && "Time interval _Dt is negative!");

    VectorXs x( _x.head(10) );
    VectorXs x_plus_delta(10);

    x_plus_delta = imu::composeOverState(x, _delta, _dt);
    _x_plus_delta.head(10) = x_plus_delta;
}

inline Eigen::VectorXs ProcessorIMU::deltaZero() const
{
    return (Eigen::VectorXs(10) << 0,0,0,  0,0,0,1,  0,0,0 ).finished(); // p, q, v
}

inline void ProcessorIMU::resetDerived()
{
    // Cast a pointer to origin IMU frame
    frame_imu_ptr_ = std::static_pointer_cast<FrameIMU>(origin_ptr_->getFramePtr());

    // Assign biases for the integration at the origin frame's biases
    acc_bias_  = frame_imu_ptr_->getAccBiasPtr()->getState(); // acc  bias
    gyro_bias_ = frame_imu_ptr_->getGyroBiasPtr()->getState(); // gyro bias
}

inline Scalar ProcessorIMU::getMaxTimeSpan() const
{
    return max_time_span_;
}

inline Scalar ProcessorIMU::getMaxBuffLength() const
{
    return max_buff_length_;
}

inline Scalar ProcessorIMU::getDistTraveled() const
{
    return dist_traveled_;
}

inline Scalar ProcessorIMU::getAngleTurned() const
{
    return angle_turned_;
}

} // namespace wolf

#endif // PROCESSOR_IMU_H
