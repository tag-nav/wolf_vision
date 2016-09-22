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

    protected:

        // Helper functions

        /**
         * @brief Extract data from the IMU and create a delta-state for one IMU step
         * @param _data
         * @param _data_cov
         * @param _dt
         */
        virtual void data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt);

        /** \brief composes a delta-state on top of another delta-state
         * \param _delta_preint the first delta-state
         * \param _delta the second delta-state
         * \param _dt the second delta-state's time delta
         * \param _delta_preint_plus_delta the delta2 composed on top of delta1. It has the format of delta-state.
         *
         *
         * _jacobian1 is A (3x9) _jacobian2 should be B (3x6) but not here..
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2
         *
         * See its definition for more comments about the inner maths.
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta_preint, const Eigen::VectorXs& _delta,
                                    const Scalar _dt, Eigen::VectorXs& _delta_preint_plus_delta);

        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta_preint, const Eigen::VectorXs& _delta,
                                    const Scalar _dt, Eigen::VectorXs& _delta_preint_plus_delta,
                                    Eigen::MatrixXs& _jacobian_delta_preint, Eigen::MatrixXs& _jacobian_delta);

        virtual void deltaMinusDelta(const Eigen::VectorXs& _delta_1, const Eigen::VectorXs& _delta_2,
                                     const Scalar _dt, Eigen::VectorXs& _delta_1_minus_delta_2);

        /** \brief composes a delta-state on top of a state
         * \param _x the initial state
         * \param _delta the delta-state
         * \param _x_plus_delta the updated state. It has the same format as the initial state.
         * \param _Dt the time interval between the origin state and the Delta
         *
         * This function implements the composition (+) so that _x2 = _x1 (+) _delta.
         */
        virtual void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _Dt,
                                Eigen::VectorXs& _x_plus_delta );



        /** \brief Delta representing the null motion
         */
        virtual Eigen::VectorXs deltaZero() const;

        virtual Motion interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts);

        void resetDerived();

        virtual ConstraintBase* createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin);

    private:


    private:

        // Casted pointer to IMU frame
        FrameIMU* frame_imu_ptr_;

        // gravity vector
        const Eigen::Vector3s gravity_;

        // Maps to the biases in the keyframe's state
        Eigen::Map<Eigen::Vector3s> bias_acc_;
        Eigen::Map<Eigen::Vector3s> bias_gyro_;

        // Maps to the received measurements
        Eigen::Map<Eigen::Vector3s> measured_acc_;
        Eigen::Map<Eigen::Vector3s> measured_gyro_;

        // Maps to pos, vel, quat of the pre-integrated delta
        Eigen::Map<Eigen::Vector3s> position_preint_;
        Eigen::Map<Eigen::Vector3s> velocity_preint_;
        Eigen::Map<Eigen::Quaternions> orientation_preint_quat_;

        // Maps to pos, vel, quat, to be used as temporaries
        Eigen::Map<const Eigen::Vector3s> p_in_1_, p_in_2_;
        Eigen::Map<Eigen::Vector3s> p_out_;
        Eigen::Map<const Eigen::Vector3s> v_in_1_, v_in_2_;
        Eigen::Map<Eigen::Vector3s> v_out_;
        Eigen::Map<const Eigen::Quaternions> q_in_1_, q_in_2_;
        Eigen::Map<Eigen::Quaternions> q_out_;


        // Helper functions to remap several magnitudes
        void remapPVQ(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta_out);
        void remapDelta(Eigen::VectorXs& _delta_out);
        void remapData(const Eigen::VectorXs& _data);

        ///Jacobians of preintegrated delta wrt IMU biases
        Eigen::Matrix3s dDp_dab_;
        Eigen::Matrix3s dDv_dab_;
        Eigen::Matrix3s dDp_dwb_;
        Eigen::Matrix3s dDv_dwb_;
        Eigen::Matrix3s dDq_dwb_;

    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};

}

/////////////////////////////////////////////////////////
// IMPLEMENTATION. Put your implementation includes here
/////////////////////////////////////////////////////////

// Wolf
#include "state_block.h"
#include "rotations.h"


namespace wolf{

inline void ProcessorIMU::data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt)
{
    assert(_data.size() == data_size_ && "Wrong data size!");

    // remap
    remapData(_data);
    remapDelta(delta_);
    // delta_ is [p,v,q]_out_

    /* MATHS of delta creation -- Sola-16
     * dp = 1/2 * (a-a_b) * dt^2 = 1/2 * dv * dt
     * dv = (a-a_b) * dt
     * dq = exp((w-w_b)*dt)
     */

    // create delta
    //Use SOLA-16 convention by default
    Eigen::Vector3s a = measured_acc_  - bias_acc_;
    Eigen::Vector3s w = measured_gyro_ - bias_gyro_;
    v_out_ = a * _dt;
    p_out_ = v_out_ * _dt / 2;
    q_out_ = v2q(w * _dt);

    //Compute jacobian of delta wrt data noise

    /* MATHS : jacobian d_n, of delta wrt noise
     * substituting a and w respectively by (a+a_n) and (w+w_n) (measurement noise is additive)
     *              an           wn
     *       dp [0.5*I*dt*dt     0      ]
     * d_n = dv [   I*dt         0      ]
     *       df [   0           Jr*dt   ] // see Sola-16
     */

    // we go the sparse way:
    Eigen::Matrix3s ddp_dan = Eigen::Matrix3s::Identity() * 0.5 * _dt * _dt;
    Eigen::Matrix3s ddv_dan = Eigen::Matrix3s::Identity() * _dt;
    //    Eigen::Matrix3s ddf_dwn = jac_SO3_right(w * _dt) * _dt; // Since wdt is small, we could use here  Jr(wdt) ~ (I - 0.5*[wdt]_x)  and go much faster.
    Eigen::Matrix3s ddf_dwn = (Eigen::Matrix3s::Identity() - 0.5 * skew(w * _dt) ) * _dt; // voila, the comment above is this

    /* Covariance is sparse:
     *       [ A  B  0
     * COV =   B' C  0
     *         0  0  D ]
     *
     * where A, B, C and D are computed below
     */
    delta_cov_.block<3,3>(0,0).noalias() = ddp_dan*_data_cov.block<3,3>(0,0)*ddp_dan.transpose(); // A
    delta_cov_.block<3,3>(0,3).noalias() = ddp_dan*_data_cov.block<3,3>(0,0)*ddv_dan.transpose(); // B
    delta_cov_.block<3,3>(3,0)           = delta_cov_.block<3,3>(0,3).transpose();                // B'
    delta_cov_.block<3,3>(3,3).noalias() = ddv_dan*_data_cov.block<3,3>(0,0)*ddv_dan.transpose(); // C
    delta_cov_.block<3,3>(6,6).noalias() = ddf_dwn*_data_cov.block<3,3>(3,3)*ddf_dwn.transpose(); // D

}

inline void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta_preint, const Eigen::VectorXs& _delta,
                                         const Scalar _dt, Eigen::VectorXs& _delta_preint_plus_delta,
                                         Eigen::MatrixXs& _jacobian_delta_preint, Eigen::MatrixXs& _jacobian_delta)
{

    remapPVQ(_delta_preint, _delta, _delta_preint_plus_delta);

    //////////////////////////////////////////////////////////
    // 1. Start by computing the Jacobians before updating the deltas. This avoids aliasing.

    /* MATHS according to Sola-16
     *
     * 1. Notation
     *
     *     We use D or Delta (with capital D) to refer to the preintegrated delta.
     *     We use d or delta (with lowercase d) to refer to the recent delta.
     *     We use p, v, q as in Dp, Dv, Dq to refer to the deltas of position, velocity, and quaternion
     *     We use f, as in Df, df, to refer to deltas of the angle vector 'phi'
     *          equivalent to the quaternion deltas (see below),
     *          that is,    Dq = Exp(Df), dq = Exp(df)
     *          and / or    DR = Exp(Df), dR = Exp(df).
     *
     * 2. Expression of the delta integration step:
     *
     *     Dp' = Dp + Dv*dt + Dq*dp   if  dp = 1/2*(a-a_b)*dt^2
     *     Dv' = Dv + Dq*dv           if  dv = (a-a_b)*dt
     *     Dq' = Dq * dq              if  dq = Exp((w-w_b)*dt)
     *
     * where (dp, dv, dq) need to be computed in data2delta(), and Dq*dx =is_equivalent_to= Dq*dx*Dq'.
     *
     * warning: All deltas (Dp, Dv, Dq) are physically interpretable: they represent the position, velocity and orientation of a body with
     * respect to a reference frame that is non-rotating and free-falling at the acceleration of gravity.
     *
     * 3. Jacobians derived from the expression above: (see Sola-16):
     *
     * With respect to Delta, D_D, is:
     *
     *   dDp'/dDp = I
     *   dDp'/dDv = I*dt
     *   dDp'/dDf = - DR * skew(dp)
     *   dDv'/dDv = I
     *   dDv'/dDf = - DR * skew(dv)
     *   dDf'/dDf = dR.tr
     *
     * which gives _jacobian_delta_preint = D_D as:
     *
     *   D_D = [ I     I*dt   -DR*skew(dp)
     *           0      I     -DR*skew(dv)
     *           0      0      dR.tr       ] // See Sola-16
     *
     * and wrt delta, D_d, is:
     *
     *   dDp'/ddp = DR
     *   dDv'/ddv = DR
     *   dDf'/ddf = I
     *
     * which gives _jacobian_delta = D_d as:
     *
     *   D_d = [ DR   0    0
     *           0    DR   0
     *           0    0    I ] // See Sola-16
     */

    // Some useful temporaries
    Eigen::Matrix3s DR      = q_in_1_.matrix();
    Eigen::Matrix3s dR      = q_in_2_.matrix();

    // Jac wrt preintegrated delta, D_D
    _jacobian_delta_preint.setIdentity(9,9);                                    // dDp'/ddp, dDv'/ddv, dDf'/ddf
    _jacobian_delta_preint.block<3,3>(0,3) = Eigen::Matrix3s::Identity() * _dt; // dDp'/ddv
    _jacobian_delta_preint.block<3,3>(0,6).noalias() = - DR * skew(p_in_2_) ;   // dDp'/ddf
    _jacobian_delta_preint.block<3,3>(3,6).noalias() = - DR * skew(v_in_2_) ;   // dDv'/ddf
    _jacobian_delta_preint.block<3,3>(6,6) =   dR.transpose();                  // dDf'/ddf

    // Jac wrt current delta, D_d
    _jacobian_delta.setZero(9,9);
    _jacobian_delta.block<3,3>(0,0) = DR;
    _jacobian_delta.block<3,3>(3,3) = DR;
    _jacobian_delta.block<3,3>(6,6) = Eigen::Matrix3s::Identity();

    /////////////////////////////////////////////////////////
    // 2. Integrate the Jacobians wrt the biases --
    // See Sola 16 -- OK Forster
    //
    // Integration of Jacobian wrt bias
    // dDp/dab += dDv/dab * dt - 0.5 * DR * dt^2
    // dDv/dab -= dR * dt
    // dDp/dwb += dDv/dwb * dt - 0.5 * DR * [a - ab]_x * dDf/dwb * dt^2
    // dDv/dwb -= DR * [a - ab]_x * dDf/dwb * dt
    // dDf/dwb = dR.tr * dDf/dwb - Jr((w - wb)*dt) * dt

    // acc and gyro measurements corrected with the estimated bias
    Eigen::Vector3s a =  measured_acc_  - bias_acc_;
    Eigen::Vector3s w =  measured_gyro_ - bias_gyro_;

    // temporaries
    Scalar dt2_2            = 0.5 * _dt * _dt;
    Eigen::Matrix3s M_tmp   = DR * skew(a) * dDq_dwb_;                            // Sola 16 -- OK Forster

    dDp_dab_.noalias()  += dDv_dab_ * _dt -  DR * dt2_2;
    dDv_dab_            -= DR * _dt;
    dDp_dwb_.noalias()  += dDv_dwb_ * _dt - M_tmp * dt2_2;
    dDv_dwb_            -= M_tmp * _dt;
    //    dDq_dwb_             = dR.transpose() * dDq_dwb_ - jac_SO3_right(w * _dt) * _dt; // See SOLA-16
    dDq_dwb_             = dR.transpose() * dDq_dwb_ - ( Eigen::Matrix3s::Identity() - 0.5*skew(w*_dt) )*_dt; // See SOLA-16

    ///////////////////////////////////////////////////////////////////////////
    // 3. Update the deltas down here to avoid aliasing in the Jacobians section
    deltaPlusDelta(_delta_preint, _delta, _dt, _delta_preint_plus_delta);

}

inline void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta_preint, const Eigen::VectorXs& _delta,
                                         const Scalar _dt, Eigen::VectorXs& _delta_preint_plus_delta)
{
    assert(_delta_preint.size() == 10 && "Wrong _delta_preint vector size");
    assert(_delta.size() == 10 && "Wrong _delta vector size");
    assert(_delta_preint_plus_delta.size() == 10 && "Wrong _delta_preint_plus_delta vector size");

    remapPVQ(_delta_preint, _delta, _delta_preint_plus_delta);
    // _delta_preint             is _in_1_
    // _delta                    is _in_2_
    // _delta_preint_plus_delta  is _out_


    /* MATHS according to Sola-16
     * Dp' = Dp + Dv*dt + 1/2*Dq*(a-a_b)*dt^2    = Dp + Dv*dt + Dq*dp   if  dp = 1/2*(a-a_b)*dt^2
     * Dv' = Dv + Dq*(a-a_b)*dt                  = Dv + Dq*dv           if  dv = (a-a_b)*dt
     * Dq' = Dq * exp((w-w_b)*dt)                = Dq * dq              if  dq = exp((w-w_b)*dt)
     *
     * where (dp, dv, dq) need to be computed in data2delta(), and Dq*dx =is_equivalent_to= Dq*dx*Dq'.
     *
     * warning: All deltas (Dp, Dv, Dq) are physically interpretable:
     * they represent the position, velocity and orientation of a body with
     * respect to a reference frame that is non-rotating and free-falling at the acceleration of gravity.
     */

    // delta pre-integration
    // For the math, use SOLA-16 convention by default
    // Note: we might be (and in fact we are) calling this fcn with the same input and output:
    //     deltaPlusDelta(delta_integrated_, delta_, dt_, delta_integrated_);
    // that is, _delta1 and _delta1_plus_delta2 point to the same memory locations.
    // Therefore, to avoid aliasing, we proceed in the order p -> v -> q
    p_out_ = p_in_1_ + v_in_1_ * _dt + q_in_1_ * p_in_2_;
    v_out_ = v_in_1_ + q_in_1_ * v_in_2_;
    q_out_ = q_in_1_ * q_in_2_;
}

inline void ProcessorIMU::deltaMinusDelta(const Eigen::VectorXs& _delta_1, const Eigen::VectorXs& _delta_2,
                                          const Scalar _dt, Eigen::VectorXs& _delta_1_minus_delta_2)
{
    assert(_delta_1.size() == 10 && "Wrong _delta_1 vector size");
    assert(_delta_2.size() == 10 && "Wrong _delta_2 vector size");
    assert(_delta_1_minus_delta_2.size() == 10 && "Wrong _delta_1_minus_delta_2 vector size");

    remapPVQ(_delta_1, _delta_2, _delta_1_minus_delta_2);
    // _delta_1                 is _in_1_
    // _delta_2                 is _in_2_
    // _delta_1_minus_delta_2   is _out_

    /* MATHS according to SOLA-16 (see deltaPlusDelta for derivation)
     */
    p_out_ = p_in_1_ + v_in_1_ * _dt - q_in_1_ * p_in_2_;
    v_out_ = v_in_1_ - q_in_1_ * v_in_2_;
    q_out_ = q_in_1_ * q_in_2_.conjugate();
}

inline void ProcessorIMU::xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _Dt,
                                     Eigen::VectorXs& _x_plus_delta)
{
    assert(_x.size() == 16 && "Wrong _x vector size");
    assert(_delta.size() == 10 && "Wrong _delta vector size");
    assert(_x_plus_delta.size() == 16 && "Wrong _x_plus_delta vector size");
    assert(_Dt >= 0 && "Time interval _Dt is negative!");

    remapPVQ(_x, _delta, _x_plus_delta);
    // _x               is _in_1_
    // _delta           is _in_2_
    // _x_plus_delta    is _out_

    Eigen::Vector3s gdt = gravity_ * _Dt;

    // state updates
    p_out_ = q_in_1_ * p_in_2_ + p_in_1_ + v_in_1_ * _Dt + gdt * _Dt / 2 ;
    v_out_ = q_in_1_ * v_in_2_ + v_in_1_ + gdt;
    q_out_ = q_in_1_ * q_in_2_;

    // bypass constant biases
    _x_plus_delta.tail(6) = _x.tail(6);
}

inline Eigen::VectorXs ProcessorIMU::deltaZero() const
{
    return (Eigen::VectorXs(10) << 0,0,0,  0,0,0,  0,0,0,1 ).finished(); // p, v, q
}

inline Motion ProcessorIMU::interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts)
{
    Motion tmp(_motion_ref);
    tmp.ts_ = _ts;
    tmp.delta_ = deltaZero();
    tmp.delta_cov_ = Eigen::MatrixXs::Zero(delta_size_, delta_size_);
    return tmp;
}


inline void ProcessorIMU::resetDerived()
{
    // Remap biases for the integration at the origin frame's biases
    frame_imu_ptr_ = (FrameIMU*)((origin_ptr_->getFramePtr()));
    new (&bias_acc_)  Eigen::Map<const Eigen::Vector3s>(frame_imu_ptr_->getBAPtr()->getVector().data()); // acc  bias
    new (&bias_gyro_) Eigen::Map<const Eigen::Vector3s>(frame_imu_ptr_->getBGPtr()->getVector().data()); // gyro bias

    // reset jacobians wrt bias
    dDp_dab_.setZero();
    dDv_dab_.setZero();
    dDp_dwb_.setZero();
    dDv_dwb_.setZero();
    dDq_dwb_.setZero();
}


inline ConstraintBase* ProcessorIMU::createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin)
{
    // return new ConstraintIMU(_feature_motion, _frame_origin);
    return nullptr;
}

inline void ProcessorIMU::remapPVQ(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta_out)
{
    new (&p_in_1_) Eigen::Map<const Eigen::Vector3s>(_delta1.data());
    new (&v_in_1_) Eigen::Map<const Eigen::Vector3s>(_delta1.data() + 3);
    new (&q_in_1_) Eigen::Map<const Eigen::Quaternions>(_delta1.data() + 6);

    new (&p_in_2_) Eigen::Map<const Eigen::Vector3s>(_delta2.data());
    new (&v_in_2_) Eigen::Map<const Eigen::Vector3s>(_delta2.data() + 3);
    new (&q_in_2_) Eigen::Map<const Eigen::Quaternions>(_delta2.data() + 6);

    new (&p_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data());
    new (&v_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data() + 3);
    new (&q_out_) Eigen::Map<Eigen::Quaternions>(_delta_out.data() + 6);
}

inline void ProcessorIMU::remapDelta(Eigen::VectorXs& _delta_out)
{
    new (&p_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data());
    new (&v_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data() + 3);
    new (&q_out_) Eigen::Map<Eigen::Quaternions>(_delta_out.data() + 6);
}

inline void ProcessorIMU::remapData(const Eigen::VectorXs& _data)
{
    new (&measured_acc_) Eigen::Map<const Eigen::Vector3s>(_data.data());
    new (&measured_gyro_) Eigen::Map<const Eigen::Vector3s>(_data.data() + 3);
}



} // namespace wolf

#endif // PROCESSOR_IMU_H
