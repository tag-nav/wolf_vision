#include <sensor_IMU.h>
#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

SensorIMU::SensorIMU(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const IntrinsicsIMU& _params) :
        SensorBase("IMU", _p_ptr, _o_ptr, std::make_shared<StateBlock>(6, false, nullptr), (Eigen::Vector6s()<<_params.a_noise,_params.a_noise,_params.a_noise,_params.w_noise,_params.w_noise,_params.w_noise).finished(), false, true),
        a_noise(_params.a_noise),
        w_noise(_params.w_noise),
        ab_initial_stdev(_params.ab_initial_stdev),
        wb_initial_stdev(_params.wb_initial_stdev),
        ab_rate_stdev(_params.ab_rate_stdev),
        wb_rate_stdev(_params.wb_rate_stdev)
{
    //
}

SensorIMU::SensorIMU(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, IntrinsicsIMUPtr _params) :
        SensorIMU(_p_ptr, _o_ptr, *_params)
{
    //
}

SensorIMU::SensorIMU(const Eigen::VectorXs& _extrinsics, IntrinsicsIMUPtr _params) :
        SensorIMU(_extrinsics, *_params)
{
    //
}

SensorIMU::SensorIMU(const Eigen::VectorXs& _extrinsics, const IntrinsicsIMU& _params) :
        SensorBase("IMU", std::make_shared<StateBlock>(_extrinsics.head(3), true), std::make_shared<StateQuaternion>(_extrinsics.tail(4), true), std::make_shared<StateBlock>(6, false, nullptr), (Eigen::Vector6s()<<_params.a_noise,_params.a_noise,_params.a_noise,_params.w_noise,_params.w_noise,_params.w_noise).finished(), false, true),
        a_noise(_params.a_noise),
        w_noise(_params.w_noise),
        ab_initial_stdev(_params.ab_initial_stdev),
        wb_initial_stdev(_params.wb_initial_stdev),
        ab_rate_stdev(_params.ab_rate_stdev),
        wb_rate_stdev(_params.wb_rate_stdev)
{
    assert(_extrinsics.size() == 7 && "Wrong extrinsics vector size! Should be 7 for 2D.");
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

    IntrinsicsIMUPtr params = std::static_pointer_cast<IntrinsicsIMU>(_intrinsics);
    SensorIMUPtr sen = std::make_shared<SensorIMU>(_extrinsics_pq, params);
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("IMU", SensorIMU)
} // namespace wolf
