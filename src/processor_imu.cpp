#include "processor_imu.h"

namespace wolf {

ProcessorIMU::ProcessorIMU() :
        ProcessorMotion(PRC_IMU, "IMU", 16, 10, 9, 6),
        frame_imu_ptr_(nullptr),
        gravity_(wolf::gravity()),
        acc_bias_(Eigen::Vector3s::Zero()),
        gyro_bias_(Eigen::Vector3s::Zero()),
        acc_measured_(nullptr),
        gyro_measured_(nullptr),
        dp_1_(nullptr), dp_2_(nullptr), dp_out_(nullptr),
        dv_1_(nullptr), dv_2_(nullptr), dv_out_(nullptr),
        dq_1_(nullptr), dq_2_(nullptr), dq_out_(nullptr)
{
    //
}

ProcessorIMU::~ProcessorIMU()
{
}

ProcessorBase* ProcessorIMU::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorIMU* prc_ptr = new ProcessorIMU();
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

} // namespace wolf


// Register in the SensorFactory
#include "processor_factory.h"
namespace wolf {
namespace
{
const bool registered_prc_imu = ProcessorFactory::get().registerCreator("IMU", ProcessorIMU::create);
}
} // namespace wolf
