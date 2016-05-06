#include "processor_imu.h"

namespace wolf {

ProcessorIMU::ProcessorIMU() :
        ProcessorMotion(PRC_IMU, 16, 10, 6)
//        sensor_imu_ptr_(nullptr),
//        capture_imu_ptr_(nullptr)
{
    setType("IMU");
}

ProcessorIMU::~ProcessorIMU()
{
}

} // namespace wolf
