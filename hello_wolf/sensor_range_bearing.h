/*
 * SensorRangeBearing.h
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#ifndef HELLO_WOLF_SENSOR_RANGE_BEARING_H_
#define HELLO_WOLF_SENSOR_RANGE_BEARING_H_

#include "base/sensor/sensor_base.h"
#include "base/utils/params_server.hpp"

namespace wolf
{
WOLF_STRUCT_PTR_TYPEDEFS(IntrinsicsRangeBearing);

struct IntrinsicsRangeBearing : public IntrinsicsBase
{
        Scalar noise_range_metres_std       = 0.05;
        Scalar noise_bearing_degrees_std    = 0.5;
    IntrinsicsRangeBearing()
    {
        //DEFINED FOR COMPATIBILITY PURPOSES. TO BE REMOVED IN THE FUTURE.
    }
    IntrinsicsRangeBearing(std::string _unique_name, const paramsServer& _server):
        IntrinsicsBase(_unique_name, _server)
    {
        noise_range_metres_std = _server.getParam<Scalar>(_unique_name + "/noise_range_metres_std", "0.05");
        noise_bearing_degrees_std = _server.getParam<Scalar>(_unique_name + "/noise_bearing_degrees_std", "0.5");
    }
};

WOLF_PTR_TYPEDEFS(SensorRangeBearing)

class SensorRangeBearing : public SensorBase
{
    public:
        SensorRangeBearing(const Eigen::VectorXs& _extrinsics, const Eigen::Vector2s& _noise_std);
        virtual ~SensorRangeBearing();

        // Factory method for high level API
        static SensorBasePtr create(const std::string& _unique_name, //
                                    const Eigen::VectorXs& _extrinsics, //
                                    const IntrinsicsBasePtr _intrinsics);
        static SensorBasePtr createAutoConf(const std::string& _unique_name, //
                                       const paramsServer& _server);
};

} /* namespace wolf */

#endif /* HELLO_WOLF_SENSOR_RANGE_BEARING_H_ */
