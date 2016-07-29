#ifndef PROCESSOR_IMU_H
#define PROCESSOR_IMU_H

// Wolf
#include "processor_motion.h"
#include "capture_imu.h"
#include "wolf.h"

// STL
#include <deque>

#include "sensor_imu.h"

namespace wolf {

class ProcessorIMU : public ProcessorMotion{
    public:
        ProcessorIMU();
        virtual ~ProcessorIMU();


        /// Bias interactions
        void setAccelerometerBias(const Eigen::Vector3s& _bias) { bias_acc_ = _bias; }
        void setGyroBias(const Eigen::Vector3s& _bias) { bias_gyro_ = _bias; }
        const Eigen::Vector3s& getAccelerometerBias() { return bias_acc_; }
        const Eigen::Vector3s& getGyroBias() { return bias_gyro_; }

        // not redefining main operations (they should be the same for all derived classes)

    protected:

//        virtual void preProcess(){}
//        virtual void postProcess(){}

        // Helper functions

        /**
         * @brief extractData Extract data from the capture_imu object and store them
         * @param _data
         * @param _data_cov
         * @param _dt
         * @param _delta
         * @param _delta_cov
         */
        virtual void data2delta(const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_cov, const Scalar _dt,
                                Eigen::VectorXs& _delta, Eigen::MatrixXs& _delta_cov)
        {
            Eigen::Vector3s measured_acc(_data.segment(0,3));        // acc  = data[0:2]
            Eigen::Vector3s measured_gyro(_data.segment(3,3));       // gyro = data[3:5]

            /// Quaternion delta
            new (&q_out_) Eigen::Map<Eigen::Quaternions>(_delta.data());
            Eigen::v2q((measured_gyro - bias_gyro_) * _dt, q_out_);

            /// Velocity delta
            new (&v_out_) Eigen::Map<Eigen::Vector3s>(_delta.data() + 4);
            v_out_ = q_out_._transformVector((measured_acc - bias_acc_) * _dt);

            /// Position delta
            new (&p_out_) Eigen::Map<Eigen::Vector3s>(_delta.data() + 7);
            p_out_ = v_out_ * dt_;
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
          assert(_delta.size() == 16 && "Wrong _delta vector size");
          assert(_x_plus_delta.size() == 16 && "Wrong _x_plus_delta vector size");

          remap(_x, _delta, _x_plus_delta);

          deltaPlusDelta(_x, _delta, _x_plus_delta);
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
          assert(_delta2.size() == 16 && "Wrong _delta2 vector size");
          assert(_delta1_plus_delta2.size() == 16 && "Wrong _delta1_plus_delta2 vector size");

          remap(_delta1, _delta2, _delta1_plus_delta2);

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

        /** \brief Computes the delta-state the goes from one delta-state to another
         * \param _delta1 the initial delta
         * \param _delta2 the final delta
         * \param _delta2_minus_delta1 the delta-state. It has the format of a delta-state.
         *
         * This function implements the composition (-) so that _delta2_minus_delta1 = _delta2 (-) _delta1.
         */
        virtual void deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                     Eigen::VectorXs& _delta2_minus_delta1) { };

        virtual Eigen::VectorXs deltaZero() const
        {
            Eigen::VectorXs tmp(16);
            tmp <<  0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0;  // p, q, v, ba, bg
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
        Eigen::Vector3s bias_acc_;
        Eigen::Vector3s bias_gyro_;

        Eigen::Map<const Eigen::Vector3s> p1_, p2_;
        Eigen::Map<Eigen::Vector3s> p_out_;
        Eigen::Map<const Eigen::Quaternions> q1_, q2_;
        Eigen::Map<Eigen::Quaternions> q_out_;
        Eigen::Map<const Eigen::Vector3s> v1_, v2_;
        Eigen::Map<Eigen::Vector3s> v_out_;
        Eigen::Map<const Eigen::Vector3s> bias_acc1_, bias_acc2_;
        Eigen::Map<Eigen::Vector3s> bias_acc_out_;
        Eigen::Map<const Eigen::Vector3s> bias_gyro1_, bias_gyro2_;
        Eigen::Map<Eigen::Vector3s> bias_gyro_out_;

        void remap(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x2, Eigen::VectorXs& _x_out);

        ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION]
        ///< (first-order propagation from *measurementCovariance*).
        Eigen::Matrix<Scalar,9,9> preint_meas_cov_;

        ///Jacobians
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasAcc_;
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasOmega_;

    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};

inline void ProcessorIMU::remap(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x2, Eigen::VectorXs& _x_out)
{
    new (&p1_) Eigen::Map<const Eigen::Vector3s>(_x1.data());
    new (&q1_) Eigen::Map<const Eigen::Quaternions>(_x1.data() + 3);
    new (&v1_) Eigen::Map<const Eigen::Vector3s>(_x1.data() + 7);
    new (&bias_acc1_) Eigen::Map<const Eigen::Vector3s>(_x1.data() + 10);
    new (&bias_gyro1_) Eigen::Map<const Eigen::Vector3s>(_x1.data() + 13);

    new (&p2_) Eigen::Map<const Eigen::Vector3s>(_x2.data());
    new (&q2_) Eigen::Map<const Eigen::Quaternions>(_x2.data()+ 3);
    new (&v2_) Eigen::Map<const Eigen::Vector3s>(_x2.data() + 7);
    new (&bias_acc2_) Eigen::Map<const Eigen::Vector3s>(_x2.data() + 10);
    new (&bias_gyro2_) Eigen::Map<const Eigen::Vector3s>(_x2.data() + 13);

    new (&p_out_) Eigen::Map<Eigen::Vector3s>(_x_out.data());
    new (&q_out_) Eigen::Map<Eigen::Quaternions>(_x_out.data() + 3);
    new (&v_out_) Eigen::Map<Eigen::Vector3s>(_x_out.data() + 7);
    new (&bias_acc_out_) Eigen::Map<const Eigen::Vector3s>(_x_out.data() + 10);
    new (&bias_gyro_out_) Eigen::Map<const Eigen::Vector3s>(_x_out.data() + 13);

}

} // namespace wolf

#endif // PROCESSOR_IMU_H
