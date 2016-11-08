
#include "sensor_gps.h"
#include "state_block.h"
#include "state_quaternion.h"

namespace wolf {

SensorGPS::SensorGPS(StateBlockPtr _p_ptr, //GPS sensor position with respect to the car frame (base frame)
                     StateBlockPtr _o_ptr, //GPS sensor orientation with respect to the car frame (base frame)
                     StateBlockPtr _bias_ptr,  //GPS sensor time bias
                     StateBlockPtr _map_p_ptr, //initial position of vehicle's frame with respect to starting point frame
                     StateBlockPtr _map_o_ptr) //initial orientation of vehicle's frame with respect to the starting point frame
        :
        SensorBase("GPS", _p_ptr, _o_ptr, _bias_ptr, 0)
{
    getStateBlockVec().resize(5);
    setStateBlockPtr(3, _map_p_ptr); // Map position
    setStateBlockPtr(4, _map_o_ptr); // Map orientation
    //
}

SensorGPS::~SensorGPS()
{
    //
}

StateBlockPtr SensorGPS::getMapPPtr() const
{
    return getStateBlockPtr(3);
}

StateBlockPtr SensorGPS::getMapOPtr() const
{
    return getStateBlockPtr(4);
}


// Define the factory method
SensorBasePtr SensorGPS::create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_p,
                              const IntrinsicsBasePtr _intrinsics)
{
    // decode extrinsics vector
    assert(_extrinsics_p.size() == 3 && "Bad extrinsics vector length. Should be 3 for 3D.");
    StateBlockPtr pos_ptr = std::make_shared<StateBlock>(_extrinsics_p, true);
    StateBlockPtr ori_ptr = nullptr;
    SensorBasePtr sen = std::make_shared<SensorGPS>(pos_ptr, ori_ptr, nullptr, nullptr, nullptr);
    sen->setName(_unique_name);
    return sen;
}

} // namespace wolf


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf {
WOLF_REGISTER_SENSOR("GPS",SensorGPS)
} // namespace wolf
