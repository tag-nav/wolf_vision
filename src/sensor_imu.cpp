#include "sensor_imu.h"

#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

SensorIMU::SensorIMU(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr) :
        SensorBase("IMU", _p_ptr, _o_ptr, nullptr, 6)
{
    //
}

SensorIMU::SensorIMU(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, IntrinsicsIMUPtr params) :
        SensorBase("IMU", _p_ptr, _o_ptr, nullptr, 6),
        gyro_noise(params->gyro_noise),
        accel_noise(params->accel_noise),
        wb_constr(params->wb_constr),
        ab_constr(params->ab_constr)
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

    IntrinsicsIMUPtr params = std::static_pointer_cast<IntrinsicsIMU>(_intrinsics);
    SensorIMUPtr sen = std::make_shared<SensorIMU>(pos_ptr, ori_ptr, params);
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("IMU", SensorIMU)
} // namespace wolf
