#include "processor_preintegrated_imu.h"

namespace wolf {

ProcessorPreintegratedIMU::ProcessorPreintegratedIMU(ProcessorType _tp) :
        ProcessorMotion(_tp, 16, 10, 6),
        sensor_imu_ptr_(nullptr),
        capture_imu_ptr_(nullptr)
{
}

ProcessorPreintegratedIMU::~ProcessorPreintegratedIMU()
{
}

} // namespace wolf
