#include "processor_imu_UnitTester.h"

namespace wolf {

ProcessorIMU_UnitTester::ProcessorIMU_UnitTester() :
        ProcessorIMU(),
        Dq_out_(nullptr)
{}

ProcessorIMU_UnitTester::~ProcessorIMU_UnitTester(){}

ProcessorBasePtr ProcessorIMU_UnitTester::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr)
{
    ProcessorIMUPtr prc_ptr = std::make_shared<ProcessorIMU>();
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}


} // namespace wolf

