#ifndef PROCESSOR_IMU_H
#define PROCESSOR_IMU_H

// Wolf
#include "processor_motion.h"
//#include "capture_imu.h"
#include "wolf.h"

// STL
#include <deque>

#include "sensor_imu.h"

namespace wolf {

class ProcessorIMU : public ProcessorMotion{
    public:
        ProcessorIMU();
        virtual ~ProcessorIMU();

        // not redefining main operations (they should be the same for all derived classes)

    protected:

        // Helper functions

        /**
         * @brief extractData Extract data from the capture_imu object and store them
         * @param _data
         * @param _data_cov
         * @param _dt
         */
        virtual void data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt)
        {
            // remap
            remapMeasurements(_data);
            remapDelta(delta_);
            remapBias(x_);
            remapPreintegratedMeasurements(delta_integrated_);

            /// Position delta
            p_out_ = velocity_preint_ * _dt;

            /// Velocity delta
            v_out_ = orientation_preint_ * ((measured_acc_ - bias_acc_) * _dt);

            /// Quaternion delta
            Eigen::v2q((measured_gyro_ - bias_gyro_) * _dt, q_out_);

          }

        /** \brief composes a delta-state on top of a state
         * \param _x the initial state
         * \param _delta the delta-state
         * \param _x_plus_delta the updated state. It has the same format as the initial state.
         *
         * This function implements the composition (+) so that _x2 = _x1 (+) _delta.
         */
        virtual void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta)
        {
          assert(_x.size() == 16 && "Wrong _x vector size");
          assert(_delta.size() == 10 && "Wrong _delta vector size");
          assert(_x_plus_delta.size() == 16 && "Wrong _x_plus_delta vector size");

          remapState(_x, _delta, _x_plus_delta);

          const Scalar dT = last_ptr_->getTimeStamp().get() - origin_ptr_->getTimeStamp().get();

          /// Position update
          p_out_ = p1_ + v1_ * dT + q1_ * p2_;

          /// Quaternion update
          q_out_ = q1_ * q2_;

          /// Velocity update
          v_out_ = v1_ + Eigen::Vector3s::Constant(0.0,0.0,9.81) * dT + q1_ * v2_;

        }

        /** \brief composes a delta-state on top of another delta-state
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2)
        {
          assert(_delta1.size() == 16 && "Wrong _delta1 vector size");
          assert(_delta2.size() == 10 && "Wrong _delta2 vector size");
          assert(_delta1_plus_delta2.size() == 16 && "Wrong _delta1_plus_delta2 vector size");

          remapDelta(_delta1, _delta2, _delta1_plus_delta2);

          p_out_ = p1_ + p2_;
          q_out_ = q1_ * q2_;
          v_out_ = v1_ + v2_;
          bias_acc_out_ = bias_acc_;
          bias_gyro_out_ = bias_gyro_;
        }

        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                    Eigen::VectorXs& _delta1_plus_delta2, Eigen::MatrixXs& _jacobian1,
                                    Eigen::MatrixXs& _jacobian2)
        {
            // TODO: all the work to be done here
        }

        // NOTE: This function is not needed -- I don't know what is it doing here, probably old code.
        //        /** \brief Computes the delta-state the goes from one delta-state to another
        //         * \param _delta1 the initial delta
        //         * \param _delta2 the final delta
        //         * \param _delta2_minus_delta1 the delta-state. It has the format of a delta-state.
        //         *
        //         * This function implements the composition (-) so that _delta2_minus_delta1 = _delta2 (-) _delta1.
        //         */
        //        virtual void deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
        //                                     Eigen::VectorXs& _delta2_minus_delta1) { };

        /** \brief Adds a delta-increment into the pre-integrated Delta-state
        * \param _delta the delta increment to add
        *
        * This function implements the pre-integrated measurements update :
        *   Delta_ik = Delta_ij (+) _delta_jk
        */
        virtual void integrateDelta()
        {
          /* In the case of IMU, updating the pre-integrated measurements
           * corresponds to adding latest delta to the pre-integrated measurements
           * with the composition rules defined by the state :
           *
           * delta_preintegrated_ik = delta_preintegrated_ij (+) delta_jk
           *
           * */
          deltaPlusDelta(delta_integrated_, delta_ , delta_integrated_);
        }

        virtual Eigen::VectorXs deltaZero() const
        {
            Eigen::VectorXs tmp(10);
            tmp <<  0,0,0,  0,0,0,1,  0,0,0;  // p, q, v
            return tmp;
        }

        virtual Motion interpolate(const Motion& _motion_ref, Motion& _motion, TimeStamp& _ts)
        {
            Motion tmp(_motion_ref);
            tmp.ts_ = _ts;
            tmp.delta_ = deltaZero();
            tmp.delta_cov_ = Eigen::MatrixXs::Zero(delta_size_, delta_size_);
            return tmp;
        }

        virtual ConstraintBase* createConstraint(FeatureBase* _feature_motion, FrameBase* _frame_origin)
        {
            // TODO: all the work to be done here
            return nullptr;
        }


    protected:
//        SensorIMU* sensor_imu_ptr_; //will contain IMU parameters
//        CaptureIMU* capture_imu_ptr_; //specific pointer to capture imu data object

    private:
        Eigen::Map<Eigen::Vector3s> bias_acc_;
        Eigen::Map<Eigen::Vector3s> bias_gyro_;

        Eigen::Map<Eigen::Vector3s> measured_acc_;
        Eigen::Map<Eigen::Vector3s> measured_gyro_;

        Eigen::Map<Eigen::Vector3s> position_preint_;
        Eigen::Map<Eigen::Quaternions> orientation_preint_;
        Eigen::Map<Eigen::Vector3s> velocity_preint_;

        Eigen::Map<const Eigen::Vector3s> p1_, p2_;
        Eigen::Map<Eigen::Vector3s> p_out_;
        Eigen::Map<const Eigen::Quaternions> q1_, q2_;
        Eigen::Map<Eigen::Quaternions> q_out_;
        Eigen::Map<const Eigen::Vector3s> v1_, v2_;
        Eigen::Map<Eigen::Vector3s> v_out_;
        Eigen::Map<const Eigen::Vector3s> bias_acc1_;
        Eigen::Map<Eigen::Vector3s> bias_acc_out_;
        Eigen::Map<const Eigen::Vector3s> bias_gyro1_;
        Eigen::Map<Eigen::Vector3s> bias_gyro_out_;

        void remapState(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_out);
        void remapDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta_out);
        void remapDelta(Eigen::VectorXs& _delta_out);
        void remapBias(const Eigen::VectorXs& _state);
        void remapMeasurements(const Eigen::VectorXs& _data);
        void remapPreintegratedMeasurements(const Eigen::VectorXs& _preint);

        ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION]
        ///< (first-order propagation from *measurementCovariance*).
        Eigen::Matrix<Scalar,9,9> preint_meas_cov_;

        ///Jacobians
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasAcc_;
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasOmega_;

    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};

inline void ProcessorIMU::remapState(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_out)
{
    new (&p1_) Eigen::Map<const Eigen::Vector3s>(_x.data());
    new (&q1_) Eigen::Map<const Eigen::Quaternions>(_x.data() + 3);
    new (&v1_) Eigen::Map<const Eigen::Vector3s>(_x.data() + 7);
    new (&bias_acc1_) Eigen::Map<const Eigen::Vector3s>(_x.data() + 10);
    new (&bias_gyro1_) Eigen::Map<const Eigen::Vector3s>(_x.data() + 13);

    new (&p2_) Eigen::Map<const Eigen::Vector3s>(_delta.data());
    new (&q2_) Eigen::Map<const Eigen::Quaternions>(_delta.data() + 3);
    new (&v2_) Eigen::Map<const Eigen::Vector3s>(_delta.data() + 7);

    new (&p_out_) Eigen::Map<Eigen::Vector3s>(_x_out.data());
    new (&q_out_) Eigen::Map<Eigen::Quaternions>(_x_out.data() + 3);
    new (&v_out_) Eigen::Map<Eigen::Vector3s>(_x_out.data() + 7);
    new (&bias_acc_out_) Eigen::Map<const Eigen::Vector3s>(_x_out.data() + 10);
    new (&bias_gyro_out_) Eigen::Map<const Eigen::Vector3s>(_x_out.data() + 13);
}

inline void ProcessorIMU::remapDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta_out)
{
    new (&p1_) Eigen::Map<const Eigen::Vector3s>(_delta1.data());
    new (&q1_) Eigen::Map<const Eigen::Quaternions>(_delta1.data() + 3);
    new (&v1_) Eigen::Map<const Eigen::Vector3s>(_delta1.data() + 7);

    new (&p2_) Eigen::Map<const Eigen::Vector3s>(_delta2.data());
    new (&q2_) Eigen::Map<const Eigen::Quaternions>(_delta2.data() + 3);
    new (&v2_) Eigen::Map<const Eigen::Vector3s>(_delta2.data() + 7);

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

inline void ProcessorIMU::remapBias(const Eigen::VectorXs& _state)
{
    new (&bias_acc_) Eigen::Map<const Eigen::Vector3s>(_state.data() + 10);
    new (&bias_gyro_) Eigen::Map<const Eigen::Vector3s>(_state.data() + 13);
}

inline void ProcessorIMU::remapMeasurements(const Eigen::VectorXs& _data)
{
    new (&measured_acc_) Eigen::Map<const Eigen::Vector3s>(_data.data());
    new (&measured_gyro_) Eigen::Map<const Eigen::Vector3s>(_data.data() + 3);
}

inline void ProcessorIMU::remapPreintegratedMeasurements(const Eigen::VectorXs& _preint)
{
    new (&position_preint_) Eigen::Map<const Eigen::Vector3s>(_preint.data());
    new (&orientation_preint_) Eigen::Map<const Eigen::Quaternions>(_preint.data() + 3);
    new (&velocity_preint_) Eigen::Map<const Eigen::Vector3s>(_preint.data() + 7);
}

} // namespace wolf

#endif // PROCESSOR_IMU_H
