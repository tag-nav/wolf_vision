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
        virtual void remapPQV(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta_out);
        virtual void remapDelta(Eigen::VectorXs& _delta_out);
        virtual void remapData(const Eigen::VectorXs& _data);

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

    // remap data
    new (&acc_measured_)    Map<const Vector3s>(_data.data());
    new (&gyro_measured_)   Map<const Vector3s>(_data.data() + 3);

    // remap delta_ is D*_out_
    new (&Dp_out_) Map<Vector3s>      (_delta.data() + 0);
    new (&Dq_out_) Map<Quaternions>   (_delta.data() + 3);
    new (&Dv_out_) Map<Vector3s>      (_delta.data() + 7);

    // acc and gyro measurements corrected with the estimated bias, times dt
    Vector3s a_dt = (acc_measured_  - _calib.head(3)) * _dt;
    Vector3s w_dt = (gyro_measured_ - _calib.tail(3)) * _dt;

    /* create delta
     *
     * MATHS of delta creation -- Sola-16
     * dp = 1/2 * (a-a_b) * dt^2 = 1/2 * dv * dt
     * dv = (a-a_b) * dt
     * dq = exp((w-w_b)*dt)
     */
    Dv_out_ = a_dt;
    Dp_out_ = Dv_out_ * _dt / 2;
    Dq_out_ = v2q(w_dt);


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

    remapPQV(_delta_preint, _delta, _delta_preint_plus_delta);

    /* This function has two stages:
     *
     * 1. Computing the Jacobians of the delta integration wrt noise: this is for the covariance propagation.
     *
     * 2. Actually integrating the deltas: this is the regular integration; it is performed at the end to avoid aliasing in the previous sections.
     *
     *
     * MATHS according to Sola-16, proof-checked against Forster-16
     *
     * A. Notation for the HELP comments
     *
     *     - We use D or Delta (with uppercase D) to refer to the preintegrated delta.
     *     - We use d or delta (with lowercase d) to refer to the recent delta.
     *     - We use p, q, v, (as in Dp, Dq, Dv) to refer to the deltas of position, quaternion, and velocity
     *     - We use o, (as in Do, do) to refer to deltas of the orientation vector equivalent to the quaternion deltas,
     *
     *          that is,    Dq = Exp(Do), dq = Exp(do), Do = Log(Dq), do = Log(dq)
     *          and / or    DR = Exp(Do), dR = Exp(do), etc.
     *
     * B. Expression of the delta integration step, D' = D (+) d:
     *
     *     Dp' = Dp + Dv*dt + Dq*dp
     *     Dv' = Dv + Dq*dv
     *     Dq' = Dq * dq
     *
     * where d = (dp, dq, dv) needs to be computed in data2delta(), and Dq*dx =is_equivalent_to= Dq*dx*Dq.conj.
     */

    /*/////////////////////////////////////////////////////////
     * 1. Jacobians for covariance propagation.
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

    // Some useful temporaries
    Matrix3s DR = Dq_.matrix(); // First  Delta, DR
    Matrix3s dR = dq_.matrix(); // Second delta, dR

    // Jac wrt preintegrated delta, D_D = dD'/dD
    _jacobian_delta_preint.block<6,6>(0,0).setIdentity(6,6);                    // dDp'/dDp, dDv'/dDv, Identities
    _jacobian_delta_preint.block<3,3>(0,3).noalias() = - DR * skew(dp_) ;       // dDp'/dDo
    _jacobian_delta_preint.block<3,3>(0,6) = Matrix3s::Identity() * _dt;        // dDp'/dDv = I*dt
    _jacobian_delta_preint.block<3,3>(3,3) =   dR.transpose();                  // dDo'/dDo
    _jacobian_delta_preint.block<3,3>(6,3).noalias() = - DR * skew(dv_) ;       // dDv'/dDo

    // Jac wrt current delta, D_d = dD'/dd
    _jacobian_delta.setIdentity(9,9);                                           //
    _jacobian_delta.block<3,3>(0,0) = DR;                                       // dDp'/ddp
    _jacobian_delta.block<3,3>(6,6) = DR;                                       // dDv'/ddv
    _jacobian_delta.block<3,3>(3,3) = Matrix3s::Identity();                     // dDo'/ddo = I



    /*//////////////////////////////////////////////////////////////////////////
     * 2. Update the deltas down here to avoid aliasing in the Jacobians section
     */
    deltaPlusDelta(_delta_preint, _delta, _dt, _delta_preint_plus_delta);

}

inline void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta_preint,
                                         const Eigen::VectorXs& _delta,
                                         const Scalar _dt,
                                         Eigen::VectorXs& _delta_preint_plus_delta)
{
    assert(_delta_preint.size() == 10 && "Wrong _delta_preint vector size");
    assert(_delta.size() == 10 && "Wrong _delta vector size");
    assert(_delta_preint_plus_delta.size() == 10 && "Wrong _delta_preint_plus_delta vector size");


    /* MATHS according to Sola-16
     * Dp' = Dp + Dv*dt + 1/2*Dq*(a-a_b)*dt^2    = Dp + Dv*dt + Dq*dp   if  dp = 1/2*(a-a_b)*dt^2
     * Dv' = Dv + Dq*(a-a_b)*dt                  = Dv + Dq*dv           if  dv = (a-a_b)*dt
     * Dq' = Dq * exp((w-w_b)*dt)                = Dq * dq              if  dq = exp((w-w_b)*dt)
     *
     * where (dp, dq, dv) need to be computed in data2delta(), and Dq*dx =is_equivalent_to= Dq*dx*Dq'.
     *
     * Note: All deltas (Dp, Dq, Dv) are physically interpretable:
     * they represent the position, velocity and orientation of a body with
     * respect to a reference frame that is non-rotating and free-falling at the acceleration of gravity.
     */

    // Note: we might be (and in fact we are) calling this fcn with the same input and output:
    //     deltaPlusDelta(delta_integrated_, delta_, dt_, delta_integrated_);
    // that is, _delta1 and _delta1_plus_delta2 point to the same memory locations.
    // Therefore, to avoid aliasing, we proceed in the order p -> v -> q
    remapPQV(_delta_preint, _delta, _delta_preint_plus_delta);
    // _delta_preint             is D*_
    // _delta                    is d*_
    // _delta_preint_plus_delta  is D*_out_
    Dp_out_ = Dp_ + Dv_ * _dt + Dq_ * dp_;
    Dv_out_ = Dv_ + Dq_ * dv_;
    Dq_out_ = Dq_ * dq_;
}

inline void ProcessorIMU::statePlusDelta(const Eigen::VectorXs& _x,
                                     const Eigen::VectorXs& _delta,
                                     const Scalar _dt,
                                     Eigen::VectorXs& _x_plus_delta)
{
    assert(_x.size() == 16 && "Wrong _x vector size");
    assert(_delta.size() == 10 && "Wrong _delta vector size");
    assert(_x_plus_delta.size() == 16 && "Wrong _x_plus_delta vector size");
    assert(_dt >= 0 && "Time interval _Dt is negative!");


    Vector3s gdt = gravity_ * _dt;

    // state updates
    remapPQV(_x, _delta, _x_plus_delta);
    // _x               is D*_
    // _delta           is d*_
    // _x_plus_delta    is D*_out_
    Dp_out_ = Dq_ * dp_ + Dp_ + Dv_ * _dt + gdt * _dt / 2 ;
    Dv_out_ = Dq_ * dv_ + Dv_ + gdt;
    Dq_out_ = Dq_ * dq_;

    // bypass constant biases
    _x_plus_delta.tail(6) = _x.tail(6);
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

inline void ProcessorIMU::remapPQV(const Eigen::VectorXs& _delta1,
                                   const Eigen::VectorXs& _delta2,
                                   Eigen::VectorXs& _delta_out)
{
    new (&Dp_) Map<const Vector3s>    (_delta1.data() + 0);
    new (&Dq_) Map<const Quaternions> (_delta1.data() + 3);
    new (&Dv_) Map<const Vector3s>    (_delta1.data() + 7);

    new (&dp_) Map<const Vector3s>    (_delta2.data() + 0);
    new (&dq_) Map<const Quaternions> (_delta2.data() + 3);
    new (&dv_) Map<const Vector3s>    (_delta2.data() + 7);

    new (&Dp_out_) Map<Vector3s>      (_delta_out.data() + 0);
    new (&Dq_out_) Map<Quaternions>   (_delta_out.data() + 3);
    new (&Dv_out_) Map<Vector3s>      (_delta_out.data() + 7);
}

inline void ProcessorIMU::remapDelta(Eigen::VectorXs& _delta_out)
{
    new (&Dp_out_) Map<Vector3s>      (_delta_out.data() + 0);
    new (&Dq_out_) Map<Quaternions>   (_delta_out.data() + 3);
    new (&Dv_out_) Map<Vector3s>      (_delta_out.data() + 7);
}

inline void ProcessorIMU::remapData(const Eigen::VectorXs& _data)
{
    new (&acc_measured_) Map<const Vector3s>(_data.data());
    new (&gyro_measured_) Map<const Vector3s>(_data.data() + 3);
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
