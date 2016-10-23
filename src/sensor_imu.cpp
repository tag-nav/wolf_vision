#include "sensor_imu.h"

#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

SensorIMU::SensorIMU(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _a_w_biases_ptr) :
//                SensorBase(SEN_IMU, "IMU", _p_ptr, _o_ptr, (_a_w_biases_ptr == nullptr) ? std::make_shared<StateBlock>(6, false) : _a_w_biases_ptr, 6)
SensorBase(SEN_IMU, "IMU", _p_ptr, _o_ptr, _a_w_biases_ptr, 6)
{
    //
}

SensorIMU::~SensorIMU()
{
    //
}

// Define the factory method
SensorBasePtr SensorIMU::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq,
                              const IntrinsicsBasePtr _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");

    StateBlockPtr pos_ptr  = std::make_shared<StateBlock>(_extrinsics_pq.head(3), true);
    StateBlockPtr ori_ptr  = std::make_shared<StateQuaternion>(_extrinsics_pq.tail(4), true);
    StateBlockPtr bias_ptr = std::make_shared<StateBlock>(6, false); // We'll have the IMU biases here

    std::shared_ptr<SensorIMU> sen = std::make_shared<SensorIMU>(pos_ptr, ori_ptr, bias_ptr);
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("IMU", SensorIMU)
} // namespace wolf
