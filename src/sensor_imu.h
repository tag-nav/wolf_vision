#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

//wolf includes
#include "sensor_base.h"

namespace wolf {

struct IntrinsicsIMU : public IntrinsicsBase
{
        // add IMU parameters here
};


class SensorIMU : public SensorBase
{

    protected:
        //add IMU parameters here

    public:
        /** \brief Constructor with arguments
         *
         * Constructor with arguments
         * \param _p_ptr StateBlock pointer to the sensor position wrt vehicle base
         * \param _o_ptr StateBlock pointer to the sensor orientation wrt vehicle base
         * \param _a_w_biases_ptr StateBlock pointer to the vector of acc and gyro biases
         *
         **/
        SensorIMU(StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _a_w_biases_ptr = nullptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~SensorIMU();

    public:
        static SensorBase* create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsBase* _intrinsics);

};

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
namespace
{
//const bool registered_imu = SensorFactory::get()->registerCreator("IMU", SensorIMU::create);
}
} // namespace wolf

#endif // SENSOR_IMU_H
