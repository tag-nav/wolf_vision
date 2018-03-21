#include <sensor_GPS_fix.h>
#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

SensorGPSFix::SensorGPSFix(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const double& _noise) :
        SensorBase("GPS FIX", _p_ptr, _o_ptr, nullptr, Eigen::VectorXs::Constant(1,_noise))
{
    //
}

SensorGPSFix::SensorGPSFix(const Eigen::VectorXs & _extrinsics, const IntrinsicsGPSFix& _intrinsics) :
        SensorBase("GPS FIX", std::make_shared<StateBlock>(_extrinsics.head(3)), std::make_shared<StateQuaternion>(_extrinsics.tail(4)), nullptr, _intrinsics.noise_std)
{
    assert((_extrinsics.size() == 2 || _extrinsics.size() == 3)
           && "Bad extrinsic vector size. Should be 2 for 2D, 3 for 3D.");
}

SensorGPSFix::SensorGPSFix(const Eigen::VectorXs & _extrinsics, IntrinsicsGPSFixPtr _intrinsics_ptr) : SensorGPSFix(_extrinsics, *_intrinsics_ptr)
{
    //
}


SensorGPSFix::~SensorGPSFix()
{
    //
}

Scalar SensorGPSFix::getNoise() const
{
    return noise_std_(0);
}

// Define the factory method
SensorBasePtr SensorGPSFix::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics,
                                 const IntrinsicsBasePtr _intrinsics)
{
    assert((_extrinsics.size() == 2 || _extrinsics.size() == 3)
            && "Bad extrinsic vector size. Should be 2 for 2D, 3 for 3D.");
    StateBlockPtr pos_ptr = std::make_shared<StateBlock>(_extrinsics, true);
    SensorGPSFixPtr sen = std::make_shared<SensorGPSFix>(pos_ptr, nullptr, 0);
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("GPS FIX", SensorGPSFix)
} // namespace wolf
