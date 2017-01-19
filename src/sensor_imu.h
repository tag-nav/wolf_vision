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
        //noise
        wolf::Scalar gyro_noise; //Rate Noise Spectral Density (same for all the axis) in deg/sec/ sqrt(Hz)
        wolf::Scalar accel_noise; //Power Spectral Density (same for all the axis) in micro_g/ sqrt(Hz)

        //Sensor Bias is optimized and stored in processorIMU
        //This is a trial to constraint how much can the bias change in 1 sec at most
        wolf::Scalar wb_constr; //gyroscope rad/sec
        wolf::Scalar ab_constr; //accelerometer micro_g/sec

        IntrinsicsIMU() :
            gyro_noise(0),
            accel_noise(0),
            wb_constr(0),
            ab_constr(0)
        {}
};

WOLF_PTR_TYPEDEFS(SensorIMU);

class SensorIMU : public SensorBase
{

    protected:
        wolf::Scalar gyro_noise; //Rate Noise Spectral Density (same for all the axis) in deg/sec/ sqrt(Hz)
        wolf::Scalar accel_noise; //Power Spectral Density (same for all the axis) in micro_g/ sqrt(Hz)

        //Sensor Bias is optimized and stored in processorIMU
        //This is a trial to constraint how much can the bias change in 1 sec at most
        wolf::Scalar wb_constr; //gyroscope rad/sec
        wolf::Scalar ab_constr; //accelerometer micro_g/sec

    public:
        /** \brief Constructor with arguments
         *
         * Constructor with arguments
         * \param _p_ptr StateBlock pointer to the sensor position wrt vehicle base
         * \param _o_ptr StateBlock pointer to the sensor orientation wrt vehicle base
         * \param _a_w_biases_ptr StateBlock pointer to the vector of acc and gyro biases
         *
         **/
        SensorIMU(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _a_w_biases_ptr = nullptr);

        /** \brief Constructor with arguments
         *
         * Constructor with arguments
         * \param _p_ptr StateBlock pointer to the sensor position wrt vehicle base
         * \param _o_ptr StateBlock pointer to the sensor orientation wrt vehicle base
         * \param params IntrinsicsIMU pointer to sensor properties
         * \param _a_w_biases_ptr StateBlock pointer to the vector of acc and gyro biases
         *
         **/
        SensorIMU(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, IntrinsicsIMUPtr params, StateBlockPtr _a_w_biases_ptr = nullptr);

        Scalar getGyroNoise() const;
        Scalar getAccelNoise() const;
        Scalar getWbConstr() const;
        Scalar getAbConstr() const;

        virtual ~SensorIMU();

    public:
        static SensorBasePtr create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsBasePtr _intrinsics = nullptr);

};

inline Scalar SensorIMU::getGyroNoise() const
{
    return gyro_noise;
}

inline Scalar SensorIMU::getAccelNoise() const
{
    return accel_noise;
}

inline Scalar SensorIMU::getWbConstr() const
{
    return wb_constr;
}

inline Scalar SensorIMU::getAbConstr() const
{
    return ab_constr;
}

} // namespace wolf

#endif // SENSOR_IMU_H
