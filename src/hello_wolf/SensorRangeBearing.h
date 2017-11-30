/*
 * SensorRangeBearing.h
 *
 *  Created on: Nov 30, 2017
 *      Author: jsola
 */

#ifndef HELLO_WOLF_SENSORRANGEBEARING_H_
#define HELLO_WOLF_SENSORRANGEBEARING_H_

#include "sensor_base.h"

namespace wolf
{

class SensorRangeBearing : public SensorBase
{
    public:
        SensorRangeBearing();
        virtual ~SensorRangeBearing();
};

} /* namespace wolf */

#endif /* HELLO_WOLF_SENSORRANGEBEARING_H_ */
