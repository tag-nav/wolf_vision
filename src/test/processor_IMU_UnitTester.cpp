#include "test/processor_IMU_UnitTester.h"

namespace wolf {

ProcessorIMU_UnitTester::ProcessorIMU_UnitTester() :
        ProcessorIMU(std::make_shared<ProcessorParamsIMU>()),
        Dq_out_(nullptr)
{}

ProcessorIMU_UnitTester::~ProcessorIMU_UnitTester(){}

} // namespace wolf

