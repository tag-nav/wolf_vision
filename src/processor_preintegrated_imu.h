#ifndef PROCESSOR_PREINTEGRATED_IMU_H
#define PROCESSOR_PREINTEGRATED_IMU_H

// Wolf
#include "processor_motion.h"
#include "capture_imu.h"
#include "sensor_imu.h"
#include "wolf.h"

// STL
#include <deque>

namespace wolf {

class ProcessorPreintegratedIMU : public ProcessorMotion{
    public:
        ProcessorPreintegratedIMU(ProcessorType _tp);
        virtual ~ProcessorPreintegratedIMU();

        // not redefining main operations (they should be the same for all derived classes)

    protected:

//        virtual void preProcess(){}
//        virtual void postProcess(){}

        // Helper functions


        /**
         * @brief extractData Extract data from the capture_imu object and store them
         * @param _capture_ptr pointer to the capture to be used for data extraction
         * @param _ts
         * @param _data
         */
        virtual void extractData(CaptureBase* _capture_ptr, TimeStamp& _ts, Eigen::VectorXs& _data);

        /** \brief composes a delta-state on top of a state
         * \param _x the initial state
         * \param _delta the delta-state
         * \param _x_plus_delta the updated state. It has the same format as the initial state.
         *
         * This function implements the composition (+) so that _x2 = _x1 (+) _delta.
         */
        virtual void xPlusDelta(const Eigen::VectorXs& _x, const Eigen::VectorXs& _delta, Eigen::VectorXs& _x_plus_delta);

        /** \brief composes a delta-state on top of another delta-state
         * \param _delta1 the first delta-state
         * \param _delta2 the second delta-state
         * \param _delta1_plus_delta2 the delta2 composed on top of delta1. It has the format of delta-state.
         *
         * This function implements the composition (+) so that _delta1_plus_delta2 = _delta1 (+) _delta2
         */
        virtual void deltaPlusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2, Eigen::VectorXs& _delta1_plus_delta2);

        /** \brief Computes the delta-state the goes from one delta-state to another
         * \param _delta1 the initial delta
         * \param _delta2 the final delta
         * \param _delta2_minus_delta1 the delta-state. It has the format of a delta-state.
         *
         * This function implements the composition (-) so that _delta2_minus_delta1 = _delta2 (-) _delta1.
         */
        virtual void deltaMinusDelta(const Eigen::VectorXs& _delta1, const Eigen::VectorXs& _delta2,
                                     Eigen::VectorXs& _delta2_minus_delta1) = 0;

    protected:
        SensorIMU* sensor_imu_ptr_; //will contain IMU parameters
        CaptureIMU* capture_imu_ptr_; //specific pointer to capture imu data object

    private:
        ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION]
        ///< (first-order propagation from *measurementCovariance*).
        Eigen::Matrix<Scalar,9,9> preint_meas_cov_;

        ///Jacobians
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasAcc_;
        Eigen::Matrix<Scalar,9,3> preintegrated_H_biasOmega_;

};

} // namespace wolf

#endif // PROCESSOR_PREINTEGRATED_IMU_H
