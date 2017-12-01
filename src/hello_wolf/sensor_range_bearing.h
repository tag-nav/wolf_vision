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
        Scalar noise_range_metres_std, noise_bearing_degrees_std;
};



WOLF_PTR_TYPEDEFS(SensorRangeBearing)

class SensorRangeBearing : public SensorBase
{
    public:
        SensorRangeBearing(const Eigen::VectorXs& _extrinsics, const Eigen::VectorXs& _intrinsics);
        virtual ~SensorRangeBearing();
        SensorBasePtr create(const std::string& _unique_name, //
                             const Eigen::VectorXs& _extrinsics_pq, //
                             const IntrinsicsBasePtr _intrinsics);
};

} /* namespace wolf */

#endif /* HELLO_WOLF_SENSOR_RANGE_BEARING_H_ */
