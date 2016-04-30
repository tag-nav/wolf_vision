#include "sensor_imu.h"

namespace wolf {

SensorIMU::SensorIMU(StateBlock* _p_ptr, StateBlock* _o_ptr) :
        SensorBase(SEN_IMU, _p_ptr, _o_ptr, nullptr, 2)
{
    setType("IMU");
}

SensorIMU::~SensorIMU()
{
    //
}

} // namespace wolf
