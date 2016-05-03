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
         *
         **/
        SensorIMU(StateBlock* _p_ptr, StateBlock* _o_ptr);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~SensorIMU();

    public:
        static SensorBase* create(const std::string& _name, const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsBase* _intrinsics);


};

} // namespace wolf

#include "state_block.h"

namespace wolf {


// Define the factory method
inline SensorBase* SensorIMU::create(const std::string& _name, const Eigen::VectorXs& _extrinsics, const IntrinsicsBase* _intrinsics)
{
    SensorBase* sen = new SensorIMU(nullptr, nullptr);
    sen->setName(_name);
    return sen;
}

} // namespace wolf

// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
namespace
{
const bool registered_imu = SensorFactory::get()->registerCreator("IMU", SensorIMU::create);
}
} // namespace wolf

#endif // SENSOR_IMU_H
