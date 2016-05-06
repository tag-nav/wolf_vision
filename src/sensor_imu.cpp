#include "sensor_imu.h"

#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

SensorIMU::SensorIMU(StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _a_w_biases_ptr) :
        SensorBase(SEN_IMU, _p_ptr, _o_ptr, (_a_w_biases_ptr == nullptr) ? new StateBlock(6, false) : _a_w_biases_ptr, 6)
{
    setType("IMU");
}

SensorIMU::~SensorIMU()
{
    //
}

// Define the factory method
SensorBase* SensorIMU::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq,
                              const IntrinsicsBase* _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_pq.size() == 7 && "Bad extrinsics vector length. Should be 7 for 3D.");
    StateBlock* pos_ptr = new StateBlock(_extrinsics_pq.head(3), true);
    StateBlock* ori_ptr = new StateQuaternion(_extrinsics_pq.tail(4), true);

    // cast instrinsics to good type and extract intrinsic vector
    //    IntrinsicsIMU* intrinsics = (IntrinsicsIMU*)((_intrinsics));
    StateBlock* bias_ptr = new StateBlock(6, false); // We'll have the IMU biases here
    SensorBase* sen = new SensorIMU(pos_ptr, ori_ptr, bias_ptr);
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf
