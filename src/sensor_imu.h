#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

//wolf includes
#include "sensor_base.h"

namespace wolf {

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
         *
         **/
        SensorIMU(StateBlock* _p_ptr, StateBlock* _o_ptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~SensorIMU();

};

} // namespace wolf

#include "sensor_factory.h"

namespace wolf {


// Define the factory method and register it in the SensorFactory
namespace
{
SensorBase* createIMU(std::string& _name, Eigen::VectorXs& _extrinsics, IntrinsicsBase* _intrinsics)
{
    SensorBase* sen = new SensorIMU(nullptr, nullptr);
    sen->setName(_name);
    return sen;
}
const bool registered_imu = SensorFactory::get()->registerCreator("IMU", createIMU);
}



} // namespace wolf

#endif // SENSOR_IMU_H
