/*
 * SensorRangeBearing.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#include "sensor_range_bearing.h"

namespace wolf
{

WOLF_PTR_TYPEDEFS(SensorRangeBearing);

SensorRangeBearing::SensorRangeBearing(const Eigen::VectorXs& _extrinsics, const Eigen::VectorXs& _intrinsics, const Eigen::Vector2s& _noise_std) :
        SensorBase("RANGE BEARING",
                   std::make_shared<StateBlock>(_extrinsics.head<2>()),
                   std::make_shared<StateAngle>(_extrinsics(2)),
                   std::make_shared<StateBlock>(_intrinsics),
                   _noise_std)
{
    assert(_extrinsics.size() == 3 && "Bad extrinsics vector size. Must be 3 for 2D");


}

SensorRangeBearing::~SensorRangeBearing()
{
    // TODO Auto-generated destructor stub
}


SensorBasePtr SensorRangeBearing::create(const std::string& _unique_name, //
        const Eigen::VectorXs& _extrinsics, //
        const IntrinsicsBasePtr _intrinsics)
{

    IntrinsicsRangeBearingPtr _intrinsics_rb = std::static_pointer_cast<IntrinsicsRangeBearing>(_intrinsics);

    SensorRangeBearingPtr sensor = std::make_shared<SensorRangeBearing>(_extrinsics, _intrinsics);

    return sensor;
}

} /* namespace wolf */


// Register in the SensorFactory
#include "sensor_factory.h"
//#include "factory.h"
namespace wolf
{
WOLF_REGISTER_SENSOR("RANGE BEARING", SensorRangeBearing)
} // namespace wolf


