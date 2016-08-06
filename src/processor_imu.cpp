#include "processor_imu.h"

namespace wolf {

ProcessorIMU::ProcessorIMU() :
        ProcessorMotion(PRC_IMU, "IMU", 16, 10, 6),
        frame_imu_ptr_(nullptr),
        gravity_(0.0, 0.0, -9.8),
        bias_acc_(nullptr),
        bias_gyro_(nullptr),
        measured_acc_(nullptr),
        measured_gyro_(nullptr),
        position_preint_(delta_integrated_.data()),
        orientation_preint_(delta_integrated_.data() + 3),
        velocity_preint_(delta_integrated_.data() + 7),
        p1_(nullptr),
        p2_(nullptr),
        p_out_(nullptr),
        q1_(nullptr),
        q2_(nullptr),
        q_out_(nullptr),
        v1_(nullptr),
        v2_(nullptr),
        v_out_(nullptr)
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
