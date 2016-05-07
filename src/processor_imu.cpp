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


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
const bool registered_prc_imu = ProcessorFactory::get()->registerCreator("IMU", ProcessorIMU::create);
}
} // namespace wolf
