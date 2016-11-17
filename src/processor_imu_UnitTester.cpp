#include "processor_imu_UnitTester.h"

namespace wolf {

ProcessorIMU_UnitTester::ProcessorIMU_UnitTester() : ProcessorIMU(){}

ProcessorIMU_UnitTester::~ProcessorIMU_UnitTester(){}

ProcessorBasePtr ProcessorIMU_UnitTester::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr)
{
    ProcessorIMU::Ptr prc_ptr = std::make_shared<ProcessorIMU>();
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
//const bool registered_prc_imu = ProcessorFactory::get().registerCreator("IMU UNIT TESTER", ProcessorIMU_UnitTester::create);
}
} // namespace wolf
