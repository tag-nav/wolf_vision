#include "frame_imu.h"
#include "constraint_base.h"
#include "trajectory_base.h"
#include "capture_base.h"
#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

FrameIMU::FrameIMU(const FrameType& _tp, const TimeStamp& _ts, const Eigen::VectorXs& _x) :
        FrameBase(_tp, _ts, std::make_shared<StateBlock>(3), std::make_shared<StateQuaternion>(), std::make_shared<StateBlock>(3))
{
    assert(_x.size() == 16 && "Wrong vector size! Must be 16.");

    resizeStateBlockVec(5); // could have done push_back, but prefer more explicit locations for the StateBlocks
    setStateBlockPtr(3, std::make_shared<StateBlock>(3)); // acc bias
    setStateBlockPtr(4, std::make_shared<StateBlock>(3)); // gyro bias
    setState(_x);
    setType("IMU");
}


FrameIMU::~FrameIMU()
{
    //      std::cout << "destructed   -F-IMU" << id() << std::endl;
}




FrameBasePtr FrameIMU::create(const FrameType & _tp,
                              const TimeStamp& _ts,
                              const Eigen::VectorXs& _x)
{
    assert(_x.size() == 16 && "Wrong state vector size. Should be 16 for an IMU with biases!");

    return std::make_shared<FrameIMU>(_tp, _ts, _x);
}

} // namespace wolf

#include "factory.h"
namespace wolf
{
WOLF_REGISTER_FRAME("IMU", FrameIMU)
} // namespace wolf
