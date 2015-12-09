//
// Created by ptirindelli on 3/12/15.
//

#include "sensor_gps.h"

SensorGPS::SensorGPS(StateBlock* _p_ptr, StateBlock* _o_ptr) :
    SensorBase(GPS_RAW, _p_ptr, _o_ptr, 0) //TODO "0" is the number of gps' parameter.
{
    //TODO initialize the gps sensor.
    std::cout << "new SensorGPS " << nodeId() << std::endl;
}

SensorGPS::~SensorGPS()
{
    //std::cout << "deleting SensorGPS " << nodeId() << std::endl;
}

