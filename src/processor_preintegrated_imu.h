#ifndef PROCESSOR_PREINTEGRATED_IMU_H
#define PROCESSOR_PREINTEGRATED_IMU_H

// Wolf
#include "processor_motion.h"
#include "wolf.h"

// STL
#include <deque>

class ProcessorPreintegratedIMU : public ProcessorMotion{
    public:
        ProcessorPreintegratedIMU(ProcessorType _tp);
        virtual ~ProcessorPreintegratedIMU();

        // not redefining main operations (they should be the same for all derived classes)

    protected:
        // Helper functions
        void integrate(Eigen::VectorXs& _data, WolfScalar _dt);
        virtual void extractData(CaptureBase* _capture_ptr, TimeStamp& _ts, Eigen::VectorXs& _data) = 0;
        virtual void data2dx(const Eigen::VectorXs& _data, WolfScalar _dt, Eigen::VectorXs& _dx);
        void pushBack(TimeStamp& _ts, Eigen::VectorXs& _dx, Eigen::VectorXs& _Dx_integral);
        void eraseFront(TimeStamp& _ts);
        virtual void plus(const Eigen::VectorXs& _x, const Eigen::VectorXs& _dx, Eigen::VectorXs& _x_plus_dx) = 0;
        virtual void minus(const Eigen::VectorXs& _x1, const Eigen::VectorXs& _x0, Eigen::VectorXs& _x1_minus_x0) = 0;

        WolfScalar computeAverageDt();
        WolfScalar getDt();

    protected:
        //ProcessorSensorIMU* sensor_imu_ptr_; //will contain IMU parameters
        //CaptureIMU* capture_imu_ptr_; //specific pointer to capture imu data object

    private:
        ///< COVARIANCE OF: [PreintPOSITION PreintVELOCITY PreintROTATION]
        ///< (first-order propagation from *measurementCovariance*).
        Eigen::Matrix<WolfScalar,9,9> preint_meas_cov_;

        ///Jacobians
        Eigen::Matrix<WolfScalar,9,3> preintegrated_H_biasAcc_;
        Eigen::Matrix<WolfScalar,9,3> preintegrated_H_biasOmega_;

};

#endif // PROCESSOR_PREINTEGRATED_IMU_H
