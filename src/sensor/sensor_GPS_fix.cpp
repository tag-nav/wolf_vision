#include "base/sensor/sensor_GPS_fix.h"
#include "base/state_block.h"
#include "base/state_quaternion.h"

namespace wolf {

SensorGPSFix::SensorGPSFix(const Eigen::VectorXs & _extrinsics, const IntrinsicsGPSFix& _intrinsics) :
        SensorBase("GPS FIX", std::make_shared<StateBlock>(_extrinsics.head(3), true), std::make_shared<StateQuaternion>(_extrinsics.tail(4), true), nullptr, _intrinsics.noise_std)
{
    assert((_extrinsics.size() == 2 || _extrinsics.size() == 3)
           && "Bad extrinsic vector size. Should be 2 for 2D, 3 for 3D.");
}

SensorGPSFix::SensorGPSFix(const Eigen::VectorXs & _extrinsics, IntrinsicsGPSFixPtr _intrinsics_ptr) :
        SensorGPSFix(_extrinsics, *_intrinsics_ptr)
{
    //
}

SensorGPSFix::~SensorGPSFix()
{
    //
}

// Define the factory method
SensorBasePtr SensorGPSFix::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics,
                                 const IntrinsicsBasePtr _intrinsics)
{
    assert((_extrinsics.size() == 2 || _extrinsics.size() == 3)
            && "Bad extrinsic vector size. Should be 2 for 2D, 3 for 3D.");
    SensorGPSFixPtr sen = std::make_shared<SensorGPSFix>(_extrinsics, std::static_pointer_cast<IntrinsicsGPSFix>(_intrinsics));
    sen->getP()->fix();
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf

// Register in the SensorFactory
#include "base/sensor/sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("GPS FIX", SensorGPSFix)
} // namespace wolf
