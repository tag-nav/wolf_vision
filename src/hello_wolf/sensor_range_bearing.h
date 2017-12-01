/*
 * SensorRangeBearing.h
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#ifndef HELLO_WOLF_SENSOR_RANGE_BEARING_H_
#define HELLO_WOLF_SENSOR_RANGE_BEARING_H_

#include "sensor_base.h"

namespace wolf
{
WOLF_STRUCT_PTR_TYPEDEFS(IntrinsicsRangeBearing);

struct IntrinsicsRangeBearing : public IntrinsicsBase
{
        Scalar noise_range_metres_std       = 0.05;
        Scalar noise_bearing_degrees_std    = 0.5;
};



WOLF_PTR_TYPEDEFS(SensorRangeBearing)

class SensorRangeBearing : public SensorBase
{
    public:
        SensorRangeBearing(const Eigen::VectorXs& _extrinsics, const Eigen::Vector2s& _noise_std);
        virtual ~SensorRangeBearing();
        SensorBasePtr create(const std::string& _unique_name, //
                             const Eigen::VectorXs& _extrinsics, //
                             const IntrinsicsBasePtr _intrinsics);
};

} /* namespace wolf */

#endif /* HELLO_WOLF_SENSOR_RANGE_BEARING_H_ */
