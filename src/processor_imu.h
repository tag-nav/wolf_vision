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
            // TODO: all the work to be done here
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
            // TODO: all the work to be done here
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
            // TODO: all the work to be done here
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
        ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION]
        ///< (first-order propagation from *measurementCovariance*).
        Eigen::Matrix<Scalar,9,9> preint_meas_cov_;

        ///Jacobians
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasAcc_;
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasOmega_;

    public:
        static ProcessorBase* create(const std::string& _unique_name, const ProcessorParamsBase* _params);
};

} // namespace wolf

#endif // PROCESSOR_IMU_H
