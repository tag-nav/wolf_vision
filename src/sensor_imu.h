#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

//wolf includes
#include "sensor_base.h"
#include <iostream>

namespace wolf {

WOLF_STRUCT_PTR_TYPEDEFS(IntrinsicsIMU);

//TODO : bias_ptr defined as intrinsics StateBlock in constructor (see SensorBase) but here we also have another intrinsics
//       This is confusing.

struct IntrinsicsIMU : public IntrinsicsBase
{
        //noise std dev
        wolf::Scalar w_noise; //standard deviation of Gyroscope noise (same for all the axis) in rad/sec/ sqrt(s)
        wolf::Scalar a_noise; //standard deviation of Acceleration noise (same for all the axis) in m/s2/sqrt(s)

        //Initial biases std dev
        wolf::Scalar ab_initial_stdev; //accelerometer micro_g/sec
        wolf::Scalar wb_initial_stdev; //gyroscope rad/sec

        // bias rate of change std dev
        Scalar ab_rate_stdev;
        Scalar wb_rate_stdev;

        IntrinsicsIMU() :
            w_noise(0.001),
            a_noise(0.04),
            ab_initial_stdev(0.00001),
            wb_initial_stdev(0.00001),
            ab_rate_stdev(0.00001),
            wb_rate_stdev(0.00001)
        {}
};

WOLF_PTR_TYPEDEFS(SensorIMU);

class SensorIMU : public SensorBase
{

    protected:
        wolf::Scalar a_noise; //Power Spectral Density (same for all the axis) in micro_g/ sqrt(Hz)
        wolf::Scalar w_noise; //Rate Noise Spectral Density (same for all the axis) in deg/sec/ sqrt(Hz)

        //This is a trial to constraint how much can the bias change in 1 sec at most
        wolf::Scalar ab_initial_stdev; //accelerometer micro_g/sec
        wolf::Scalar wb_initial_stdev; //gyroscope rad/sec
        wolf::Scalar ab_rate_stdev; //accelerometer micro_g/sec
        wolf::Scalar wb_rate_stdev; //gyroscope rad/sec

    public:

        /** \brief Constructor with arguments
         *
         * Constructor with arguments
         * \param _p_ptr StateBlock pointer to the sensor position wrt vehicle base
         * \param _o_ptr StateBlock pointer to the sensor orientation wrt vehicle base
         * \param params IntrinsicsIMU pointer to sensor properties
         * \param _a_w_biases_ptr StateBlock pointer to the vector of acc and gyro biases
         *
         **/
        SensorIMU(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, IntrinsicsIMUPtr params);

        Scalar getGyroNoise() const;
        Scalar getAccelNoise() const;
        Scalar getWbInitialStdev() const;
        Scalar getAbInitialStdev() const;
        Scalar getWbRateStdev() const;
        Scalar getAbRateStdev() const;

        virtual ~SensorIMU();

    public:
        static SensorBasePtr create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsBasePtr _intrinsics = nullptr);

};

inline Scalar SensorIMU::getGyroNoise() const
{
    return w_noise;
}

inline Scalar SensorIMU::getAccelNoise() const
{
    return a_noise;
}

inline Scalar SensorIMU::getWbInitialStdev() const
{
    return wb_initial_stdev;
}

inline Scalar SensorIMU::getAbInitialStdev() const
{
    return ab_initial_stdev;
}

inline Scalar SensorIMU::getWbRateStdev() const
{
    return wb_rate_stdev;
}

inline Scalar SensorIMU::getAbRateStdev() const
{
    return ab_rate_stdev;
}

} // namespace wolf

#endif // SENSOR_IMU_H
