#ifndef PROCESSOR_IMU_H
#define PROCESSOR_IMU_H

// Wolf
#include "processor_motion.h"
#include "wolf.h"
#include "frame_imu.h"
#include "sensor_imu.h"
#include "state_block.h"

// STL
#include <deque>


namespace wolf {

class ProcessorIMU : public ProcessorMotion{
    public:
        ProcessorIMU();
        virtual ~ProcessorIMU();

    public:
        const Eigen::Vector3s& getGravity() const;

    protected:

        // Helper functions

        /**
         * @brief extractData Extract data from the capture_imu object and store them
         * @param _data
         * @param _data_cov
         * @param _dt
         */
        virtual void data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt);

        /** \brief composes a delta-state on top of another delta-state
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                    Eigen::VectorXs& _delta1_plus_delta2);

        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                    Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                    Eigen::MatrixXs& _jacobian2);

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


        /** \brief Adds a delta-increment into the pre-integrated Delta-state
        * \param _delta the delta increment to add
        *
        * This function implements the pre-integrated measurements update :
        *   Delta_ik = Delta_ij (+) _delta_jk
        */
        virtual void integrateDelta();

        virtual Eigen::VectorXs deltaZero() const;

        virtual Motion interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts);

        void resetDerived();

        virtual ConstraintBase* createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin);

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

        // Maps to the pos, quat and vel of the pre-integrated delta
        Eigen::Map<Eigen::Vector3s> position_preint_;
        Eigen::Map<Eigen::Quaternions> orientation_preint_;
        Eigen::Map<Eigen::Vector3s> velocity_preint_;

        // Maps to pos, quat, vel, to be used as temporaries
        Eigen::Map<const Eigen::Vector3s> p_in_1_, p_in_2_;
        Eigen::Map<Eigen::Vector3s> p_out_;
        Eigen::Map<const Eigen::Quaternions> q_in_1_, q_in_2_;
        Eigen::Map<Eigen::Quaternions> q_out_;
        Eigen::Map<const Eigen::Vector3s> v_in_1_, v_in_2_;
        Eigen::Map<Eigen::Vector3s> v_out_;


        // Helper functions to remap several magnitudes
        void remapPQV(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta_out);
        void remapDelta(Eigen::VectorXs& _delta_out);
        void remapData(const Eigen::VectorXs& _data);

        ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION]
        ///< (first-order propagation from *measurementCovariance*).
        Eigen::Matrix<Scalar,9,9> preint_meas_cov_;

        ///Jacobians
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasAcc_;
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasOmega_;

    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};

inline const Eigen::Vector3s& ProcessorIMU::getGravity() const
{
    return gravity_;
}

inline void ProcessorIMU::data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt)
{
    assert(_data.size() == data_size_ && "Wrong data size!");

    // remap
    remapData(_data);
    remapDelta(delta_);
    // delta_ is _out_

    /*  MATHS of delta creation -- forster-15
        dq = exp(wdt)
        dv = Dq * (a*dt) * Dq'
        dp = (3/2)*dv*dt
    */
    // create delta
    p_out_ = velocity_preint_ * _dt;
    Eigen::v2q((measured_gyro_ - bias_gyro_) * _dt, q_out_); // q_out_
    v_out_ = orientation_preint_ * ((measured_acc_ - bias_acc_) * _dt);

    /* MATHS of delta creation -- Sola-16
     * dp = 1/2 * (a-a_b) * dt^2
     * dv = (a-a_b) * dt
     * dq = exp((w-w_b)*dt)
     */
    // According to Sola-16
//    v_out_ = (measured_acc_ - bias_acc_) * _dt;
//    p_out_ = v_out_ * _dt / 2;
//    Eigen::v2q((measured_gyro_ - bias_gyro_) * _dt, q_out_); // q_out_

}

inline void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                         Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                         Eigen::MatrixXs& _jacobian2)
{
    deltaPlusDelta(_delta1, _delta2, _delta1_plus_delta2);
    // TODO: all the work to be done here about Jacobians
}

inline void ProcessorIMU::deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                         Eigen::VectorXs& _delta1_plus_delta2)
{
    assert(_delta1.size() == 10 && "Wrong _delta1 vector size");
    assert(_delta2.size() == 10 && "Wrong _delta2 vector size");
    assert(_delta1_plus_delta2.size() == 10 && "Wrong _delta1_plus_delta2 vector size");

    remapPQV(_delta1, _delta2, _delta1_plus_delta2);
    // _delta1              is _in_1_
    // _delta2              is _in_2_
    // _delta1_plus_delta2  is _out_

    /* MATHS of delta pre-integration according to Forster-15
        Dp_ = Dp + (3/2)*dv*dt           = Dp + dp
        Dq_ = Dq * exp(w*dt)             = Dq * dq
        Dv_ = Dv + Dq * exp(a*dt) * Dq'  = Dv + dv

        warning : only Dq represents a real preintegrated rotation (physically)
                    Dv and Dp are not physical representations of preintegrated velocity and position
    */

    // delta pre-integration
    p_out_ = p_in_1_ + p_in_2_;
    q_out_ = q_in_1_ * q_in_2_;
    v_out_ = v_in_1_ + v_in_2_;

    /* MATHS according to Sola-16
     * Dp' = Dp + Dv*dt + 1/2*Dq*(a-a_b)*dt^2    = Dp + Dv*dt + Dq*dp
     * Dq' = Dq * exp((w-w_b)*dt)                = Dq * dq
     * Dv' = Dv + Dq*(a-a_b)^2                   = Dv + Dq*dv
     *
     * where (dp, dv, dq) need to be computed in data2delta(), and Dq*dx =is_equivalent_to= Dq*dx*Dq'.
     *
     * warning: All deltas (Dp, Dv, Dq) are physically interpretable: they represent the position, velocity and orientation of a body with
     * respect to a reference frame that is non-rotating and free-falling at the acceleration of gravity.
     *
     * To implement this, we need to accept Dt in the API. Important notes:
     *  - Dt is the total time increment of the pre-integrated Delta, _delta2, as opposed to dt, which is the IMU time step.
     *    - During regular delta composition, Dt is the time interval of the second Delta.
     *    - During pre-integration, we are integrating just the last IMU step, and therefore in such cases we have Dt = dt.
     */

     // delta pre-integration
     p_out_ = p_in_1_ + v_in_1_ * dt_ + q_in_1_ * p_in_2_;
     q_out_ = q_in_1_ * q_in_2_;
     v_out_ = v_in_1_ + q_in_1_ * v_in_2_;

}

inline void ProcessorIMU::xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, const Scalar _Dt,
                                     Eigen::VectorXs& _x_plus_delta)
{
    assert(_x.size() == 16 && "Wrong _x vector size");
    assert(_delta.size() == 10 && "Wrong _delta vector size");
    assert(_x_plus_delta.size() == 16 && "Wrong _x_plus_delta vector size");
    assert(_Dt > 0 && "Time interval _Dt is not positive!");

    remapPQV(_x, _delta, _x_plus_delta);
    // _x               is _in_1_
    // _delta           is _in_2_
    // _x_plus_delta    is _out_

    Eigen::Vector3s gdt = gravity_ * _Dt;

    // state updates
    p_out_ = q_in_1_ * p_in_2_ + p_in_1_ + v_in_1_ * _Dt + gdt * _Dt / 2 ;
    q_out_ = q_in_1_ * q_in_2_;
    v_out_ = q_in_1_ * v_in_2_ + v_in_1_ + gdt;

    // bypass constant biases
    _x_plus_delta.tail(6) = _x.tail(6);
}

inline void ProcessorIMU::integrateDelta()
{
    /* In the case of IMU, updating the pre-integrated measurements
     * corresponds to adding latest delta to the pre-integrated measurements
     * with the composition rules defined by the state :
     *
     * delta_preintegrated_ik = delta_preintegrated_ij (+) delta_jk
     *
     * */
    deltaPlusDelta(delta_integrated_, delta_, delta_integrated_);
}

inline Eigen::VectorXs ProcessorIMU::deltaZero() const
{
    return (Eigen::VectorXs(10) << 0,0,0,  0,0,0,1,  0,0,0 ).finished(); // p, q, v
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
}


inline ConstraintBase* ProcessorIMU::createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin)
{
    // return new ConstraintIMU(_feature_motion, _frame_origin);
    return nullptr;
}

inline void ProcessorIMU::remapPQV(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta_out)
{
    new (&p_in_1_) Eigen::Map<const Eigen::Vector3s>(_delta1.data());
    new (&q_in_1_) Eigen::Map<const Eigen::Quaternions>(_delta1.data() + 3);
    new (&v_in_1_) Eigen::Map<const Eigen::Vector3s>(_delta1.data() + 7);

    new (&p_in_2_) Eigen::Map<const Eigen::Vector3s>(_delta2.data());
    new (&q_in_2_) Eigen::Map<const Eigen::Quaternions>(_delta2.data() + 3);
    new (&v_in_2_) Eigen::Map<const Eigen::Vector3s>(_delta2.data() + 7);

    new (&p_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data());
    new (&q_out_) Eigen::Map<Eigen::Quaternions>(_delta_out.data() + 3);
    new (&v_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data() + 7);
}

inline void ProcessorIMU::remapDelta(Eigen::VectorXs& _delta_out)
{
    new (&p_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data());
    new (&q_out_) Eigen::Map<Eigen::Quaternions>(_delta_out.data() + 3);
    new (&v_out_) Eigen::Map<Eigen::Vector3s>(_delta_out.data() + 7);
}

inline void ProcessorIMU::remapData(const Eigen::VectorXs& _data)
{
    new (&measured_acc_) Eigen::Map<const Eigen::Vector3s>(_data.data());
    new (&measured_gyro_) Eigen::Map<const Eigen::Vector3s>(_data.data() + 3);
}

} // namespace wolf

#endif // PROCESSOR_IMU_H
