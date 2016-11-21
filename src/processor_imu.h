#ifndef PROCESSOR_IMU_H
#define PROCESSOR_IMU_H

// Wolf
#include "processor_motion.h"
#include "frame_imu.h"


namespace wolf {

class ProcessorIMU : public ProcessorMotion{
    public:
        ProcessorIMU();
        virtual ~ProcessorIMU();

        //void getJacobians(Eigen::Matrix3s& _dDp_dab, Eigen::Matrix3s& _dDv_dab, Eigen::Matrix3s& _dDp_dwb, Eigen::Matrix3s& _dDv_dwb, Eigen::Matrix3s& _dDq_dwb);

    protected:
        virtual void data2delta(const Eigen::VectorXs& _data,
                                const Eigen::MatrixXs& _data_cov,
                                const Scalar _dt);
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta_preint,
                                    const Eigen::VectorXs& _delta,
                                    const Scalar _dt,
                                    Eigen::VectorXs& _delta_preint_plus_delta);
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta_preint,
                                    const Eigen::VectorXs& _delta,
                                    const Scalar _dt,
                                    Eigen::VectorXs& _delta_preint_plus_delta,
                                    Eigen::MatrixXs& _jacobian_delta_preint,
                                    Eigen::MatrixXs& _jacobian_delta);
        virtual void xPlusDelta(const Eigen::VectorXs& _x,
                                const Eigen::VectorXs& _delta,
                                const Scalar _dt,
                                Eigen::VectorXs& _x_plus_delta );
        virtual Eigen::VectorXs deltaZero() const;
        virtual Motion interpolate(const Motion& _motion_ref,
                                   Motion& _motion,
                                   TimeStamp& _ts);
        virtual ConstraintBasePtr emplaceConstraint(FeatureBasePtr _feature_motion,
                                                   FrameBasePtr _frame_origin);
        void resetDerived();

    protected:

        // Casted pointer to IMU frame
        std::shared_ptr<FrameIMU> frame_imu_ptr_;

        // gravity vector
        const Eigen::Vector3s gravity_;

        // Biases in the first keyframe's state for pre-integration
        Eigen::Vector3s acc_bias_;
        Eigen::Vector3s gyro_bias_;

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

        ///Jacobians of preintegrated delta wrt IMU biases
        Eigen::Matrix3s dDp_dab_;
        Eigen::Matrix3s dDv_dab_;
        Eigen::Matrix3s dDp_dwb_;
        Eigen::Matrix3s dDv_dwb_;
        Eigen::Matrix3s dDq_dwb_;

        // Helper functions to remap several magnitudes
        virtual void remapPQV(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta_out);
        virtual void remapDelta(Eigen::VectorXs& _delta_out);
        virtual void remapData(const Eigen::VectorXs& _data);


    public:
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

inline void ProcessorIMU::data2delta(const Eigen::VectorXs& _data,
                                     const Eigen::MatrixXs& _data_cov,
                                     const Scalar _dt)
{
    assert(_data.size() == data_size_ && "Wrong data size!");

    // remap
    remapData(_data);
    remapDelta(delta_);
    // delta_ is D*_out_

    /* MATHS of delta creation -- Sola-16
     * dp = 1/2 * (a-a_b) * dt^2 = 1/2 * dv * dt
     * dv = (a-a_b) * dt
     * dq = exp((w-w_b)*dt)
     */

    // acc and gyro measurements corrected with the estimated bias
    Eigen::Vector3s a = acc_measured_  - acc_bias_;
    Eigen::Vector3s w = gyro_measured_ - gyro_bias_;

    // create delta
    Dv_out_ = a * _dt;
    Dp_out_ = Dv_out_ * _dt / 2;
    Dq_out_ = v2q(w * _dt);

    //Compute jacobian of delta wrt data noise

    /* MATHS : jacobian dd_dn, of delta wrt noise
     * substituting a and w respectively by (a+a_n) and (w+w_n) (measurement noise is additive)
     *                 an           wn
     *         dp [ 0.5*I*dt*dt     0     ]
     * dd_dn = do [    0           Jr*dt  ]
     *         dv [    I*dt         0     ] // see Sola-16
     */

    // we go the sparse way:
    Eigen::Matrix3s ddv_dan = Eigen::Matrix3s::Identity() * _dt;
    Eigen::Matrix3s ddp_dan = ddv_dan * _dt / 2;
    //    Eigen::Matrix3s ddo_dwn = jac_SO3_right(w * _dt) * _dt; // Since w*dt is small, we could use here  Jr(wdt) ~ (I - 0.5*[wdt]_x)  and go much faster.
    Eigen::Matrix3s ddo_dwn = (Eigen::Matrix3s::Identity() - 0.5 * skew(w * _dt) ) * _dt; // voila, the comment above is this

    /* Covariance is sparse:
     *       [ Cpp   0   Cpv
     * COV =    0   Coo   0
     *         Cpv'  0   Cvv ]
     *
     * where Cpp, Cpv, Coo and Cvv are computed below
     */
    delta_cov_.block<3,3>(0,0).noalias() = ddp_dan*_data_cov.block<3,3>(0,0)*ddp_dan.transpose(); // Cpp = ddp_dan * Caa * ddp_dan'
    delta_cov_.block<3,3>(0,6).noalias() = ddp_dan*_data_cov.block<3,3>(0,0)*ddv_dan.transpose(); // Cpv = ddp_dan * Caa * ddv_dan'
    delta_cov_.block<3,3>(3,3).noalias() = ddo_dwn*_data_cov.block<3,3>(3,3)*ddo_dwn.transpose(); // Coo = ddo_dwn * Cww * ddo_dwn'
    delta_cov_.block<3,3>(6,0)           = delta_cov_.block<3,3>(0,6).transpose();                // Cvp = Cpv'
    delta_cov_.block<3,3>(6,6).noalias() = ddv_dan*_data_cov.block<3,3>(0,0)*ddv_dan.transpose(); // Cvv = ddv_dan * Caa * ddv_dan'

}

inline void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta_preint,
                                         const Eigen::VectorXs& _delta,
                                         const Scalar _dt,
                                         Eigen::VectorXs& _delta_preint_plus_delta,
                                         Eigen::MatrixXs& _jacobian_delta_preint,
                                         Eigen::MatrixXs& _jacobian_delta)
{

    remapPQV(_delta_preint, _delta, _delta_preint_plus_delta);

    /* This function has four stages:
     *
     * 1. Computing the Jacobians of the delta integration wrt noise: this is for the covariance propagation.
     *
     * 2. Integrating the Jacobians wrt the biases: this is for Delta correction upon bias changes.
     *
     * 3. Actually integrating the deltas: this is the regular integration; it is performed at the end to avoid aliasing in the previous sections.
     *
     *
     * MATHS according to Sola-16, proof-checked against Forster-16
     *
     * A. Notation for the HELP comments
     *
     *     - We use D or Delta (with uppercase D) to refer to the preintegrated delta.
     *     - We use d or delta (with lowercase d) to refer to the recent delta.
     *     - We use p, q, v, (as in Dp, Dq, Dv) to refer to the deltas of position, quaternion, and velocity
     *     - We use f, (as in Do, do) to refer to deltas of the orientation vector equivalent to the quaternion deltas,
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
     * where d = (dp, dq, dv) needs to be computed in data2delta(), and Dq*dx =is_equivalent_to= Dq*dx*Dq'.
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
    Eigen::Matrix3s DR = Dq_.matrix(); // First  Delta, DR
    Eigen::Matrix3s dR = dq_.matrix(); // Second delta, dR

    // Jac wrt preintegrated delta, D_D = dD'/dD
//    _jacobian_delta_preint.block<6,6>(0,0).setIdentity(6,6);                     // dDp'/dDp, dDv'/dDv, Identities
    _jacobian_delta_preint.block<3,3>(0,3).noalias() = - DR * skew(dp_) ;       // dDp'/dDo
    _jacobian_delta_preint.block<3,3>(0,6) = Eigen::Matrix3s::Identity() * _dt; // dDp'/dDv = I*dt
    _jacobian_delta_preint.block<3,3>(3,3) =   dR.transpose();                  // dDo'/dDo
    _jacobian_delta_preint.block<3,3>(6,3).noalias() = - DR * skew(dv_) ;       // dDv'/dDo

    // Jac wrt current delta, D_d = dD'/dd
//    _jacobian_delta.setIdentity(9,9);                                           //
    _jacobian_delta.block<3,3>(0,0) = DR;                                       // dDp'/ddp
    _jacobian_delta.block<3,3>(6,6) = DR;                                       // dDv'/ddv
//    _jacobian_delta.block<3,3>(3,3) = Eigen::Matrix3s::Identity();        // dDo'/ddo = I



     /*////////////////////////////////////////////////////////
      * 2. Integrate the Jacobians wrt the biases --
      * See Sola 16 -- OK Forster
      *
      * Integration of Jacobian wrt bias
      * dDp/dab += dDv/dab * dt - 0.5 * DR * dt^2
      * dDv/dab -= DR * dt
      * dDp/dwb += dDv/dwb * dt - 0.5 * DR * [a - ab]_x * dDo/dwb * dt^2
      * dDv/dwb -= DR * [a - ab]_x * dDo/dwb * dt
      * dDo/dwb  = dR.tr * dDo/dwb - Jr((w - wb)*dt) * dt
      */

    // acc and gyro measurements corrected with the estimated bias
    Eigen::Vector3s a = acc_measured_  - acc_bias_;
    Eigen::Vector3s w = gyro_measured_ - gyro_bias_;

    // temporaries
    Scalar dt2_2            = 0.5 * _dt * _dt;
    Eigen::Matrix3s M_tmp   = DR * skew(a) * dDq_dwb_;

    dDp_dab_.noalias()  += dDv_dab_ * _dt -  DR * dt2_2;
    dDv_dab_            -= DR * _dt;
    dDp_dwb_.noalias()  += dDv_dwb_ * _dt - M_tmp * dt2_2;
    dDv_dwb_            -= M_tmp * _dt;
    //    dDq_dwb_       = dR.transpose() * dDq_dwb_ - jac_SO3_right(w * _dt) * _dt; // See SOLA-16 -- we'll use small angle aprox below:
    dDq_dwb_             = dR.transpose() * dDq_dwb_ - ( Eigen::Matrix3s::Identity() - 0.5*skew(w*_dt) )*_dt; // Small angle aprox of right Jacobian above


    /*//////////////////////////////////////////////////////////////////////////
     * 3. Update the deltas down here to avoid aliasing in the Jacobians section
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

    remapPQV(_delta_preint, _delta, _delta_preint_plus_delta);
    // _delta_preint             is *_1_
    // _delta                    is *_2_
    // _delta_preint_plus_delta  is *_out_

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
    Dp_out_ = Dp_ + Dv_ * _dt + Dq_ * dp_;
    Dv_out_ = Dv_ + Dq_ * dv_;
    Dq_out_ = Dq_ * dq_;
}

inline void ProcessorIMU::xPlusDelta(const Eigen::VectorXs& _x,
                                     const Eigen::VectorXs& _delta,
                                     const Scalar _dt,
                                     Eigen::VectorXs& _x_plus_delta)
{
    assert(_x.size() == 16 && "Wrong _x vector size");
    assert(_delta.size() == 10 && "Wrong _delta vector size");
    assert(_x_plus_delta.size() == 16 && "Wrong _x_plus_delta vector size");
    assert(_dt >= 0 && "Time interval _Dt is negative!");

    remapPQV(_x, _delta, _x_plus_delta);
    // _x               is _1_
    // _delta           is _2_
    // _x_plus_delta    is _out_

    Eigen::Vector3s gdt = gravity_ * _dt;

    // state updates
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

inline Motion ProcessorIMU::interpolate(const Motion& _motion_ref,
                                        Motion& _motion,
                                        TimeStamp& _ts)
{
    Motion tmp(_motion_ref);
    tmp.ts_ = _ts;
    tmp.delta_ = deltaZero();
    tmp.delta_cov_ = Eigen::MatrixXs::Zero(delta_size_, delta_size_);
    return tmp;
}


inline void ProcessorIMU::resetDerived()
{
    // Cast a pointer to origin IMU frame
    frame_imu_ptr_ = std::static_pointer_cast<FrameIMU>(origin_ptr_->getFramePtr());

    // Assign biases for the integration at the origin frame's biases
    acc_bias_  = frame_imu_ptr_->getAccBiasPtr()->getVector(); // acc  bias
    gyro_bias_ = frame_imu_ptr_->getGyroBiasPtr()->getVector(); // gyro bias

    // reset jacobians wrt bias
    dDp_dab_.setZero();
    dDv_dab_.setZero();
    dDp_dwb_.setZero();
    dDv_dwb_.setZero();
    dDq_dwb_.setZero();
}

//<<<<<<< 13d73e2d3fde7d0c8b220e57319ec25cb5426eed
//inline ConstraintBasePtr ProcessorIMU::createConstraint(FeatureBasePtr _feature_motion,
//                                                        FrameBasePtr _frame_origin)
//{
//    auto ftr_imu = std::static_pointer_cast<FeatureIMU>(_feature_motion);
//    auto frm_imu = std::static_pointer_cast<FrameIMU>(_frame_origin);
//    auto ctr_imu = std::make_shared<ConstraintIMU>(ftr_imu, frm_imu);
//=======
inline ConstraintBasePtr ProcessorIMU::emplaceConstraint(FeatureBasePtr _feature_motion, FrameBasePtr _frame_origin)
{
    FeatureIMU::Ptr ftr_imu = std::static_pointer_cast<FeatureIMU>(_feature_motion);
    FrameIMU::Ptr frm_imu = std::static_pointer_cast<FrameIMU>(_frame_origin);
    ConstraintIMU::Ptr ctr_imu = std::make_shared<ConstraintIMU>(ftr_imu, frm_imu);

    _feature_motion->addConstraint(ctr_imu);
    _frame_origin->addConstrainedBy(ctr_imu);

//>>>>>>> Rename ProcessorXx::createConstraint --> emplaceconstraint
    return ctr_imu;
}

inline void ProcessorIMU::remapPQV(const Eigen::VectorXs& _delta1,
                                   const Eigen::VectorXs& _delta2,
                                   Eigen::VectorXs& _delta_out)
{
    new (&Dp_) Eigen::Map<const Eigen::Vector3s>    (_delta1.data() + 0);
    new (&Dq_) Eigen::Map<const Eigen::Quaternions> (_delta1.data() + 3);
    new (&Dv_) Eigen::Map<const Eigen::Vector3s>    (_delta1.data() + 7);

    new (&dp_) Eigen::Map<const Eigen::Vector3s>    (_delta2.data() + 0);
    new (&dq_) Eigen::Map<const Eigen::Quaternions> (_delta2.data() + 3);
    new (&dv_) Eigen::Map<const Eigen::Vector3s>    (_delta2.data() + 7);

    new (&Dp_out_) Eigen::Map<Eigen::Vector3s>      (_delta_out.data() + 0);
    new (&Dq_out_) Eigen::Map<Eigen::Quaternions>   (_delta_out.data() + 3);
    new (&Dv_out_) Eigen::Map<Eigen::Vector3s>      (_delta_out.data() + 7);
}

inline void ProcessorIMU::remapDelta(Eigen::VectorXs& _delta_out)
{
    new (&Dp_out_) Eigen::Map<Eigen::Vector3s>      (_delta_out.data() + 0);
    new (&Dq_out_) Eigen::Map<Eigen::Quaternions>   (_delta_out.data() + 3);
    new (&Dv_out_) Eigen::Map<Eigen::Vector3s>      (_delta_out.data() + 7);
}

inline void ProcessorIMU::remapData(const Eigen::VectorXs& _data)
{
    new (&acc_measured_) Eigen::Map<const Eigen::Vector3s>(_data.data());
    new (&gyro_measured_) Eigen::Map<const Eigen::Vector3s>(_data.data() + 3);
}

/*void ProcessorIMU::getJacobians(Eigen::Matrix3s& _dDp_dab, Eigen::Matrix3s& _dDv_dab, Eigen::Matrix3s& _dDp_dwb, Eigen::Matrix3s& _dDv_dwb, Eigen::Matrix3s& _dDq_dwb)
{
    _dDp_dab = dDp_dab_;
    _dDv_dab = dDv_dab_;
    _dDp_dwb = dDp_dwb_;
    _dDv_dwb = dDv_dwb_;
    _dDq_dwb = dDq_dwb_;
}*/

} // namespace wolf

#endif // PROCESSOR_IMU_H
