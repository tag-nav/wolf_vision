/*
 * SensorRangeBearing.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#include "sensor_range_bearing.h"
#include "state_angle.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(SensorRangeBearing);

SensorRangeBearing::SensorRangeBearing(const Eigen::VectorXs& _extrinsics, const Eigen::Vector2s& _noise_std) :
        SensorBase("RANGE BEARING",
                   std::make_shared<StateBlock>(_extrinsics.head<2>()),
                   std::make_shared<StateAngle>(_extrinsics(2)),
                   nullptr,
                   _noise_std)
{
    assert(_extrinsics.size() == 3 && "Bad extrinsics vector size. Must be 3 for 2D");
}

SensorRangeBearing::~SensorRangeBearing()
{
    //
}


SensorBasePtr SensorRangeBearing::create(const std::string& _unique_name, //
        const Eigen::VectorXs& _extrinsics, //
        const IntrinsicsBasePtr _intrinsics)
{

    IntrinsicsRangeBearingPtr _intrinsics_rb = std::static_pointer_cast<IntrinsicsRangeBearing>(_intrinsics);

    Eigen::Vector2s noise_std(_intrinsics_rb->noise_range_metres_std, toRad(_intrinsics_rb->noise_bearing_degrees_std));

    SensorRangeBearingPtr sensor = std::make_shared<SensorRangeBearing>(_extrinsics, noise_std);

    return sensor;
}

} /* namespace wolf */


// Register in the SensorFactory
#include "sensor_factory.h"
namespace wolf
{
WOLF_REGISTER_SENSOR("RANGE BEARING", SensorRangeBearing)
} // namespace wolf


