//
// Created by ptirindelli on 3/12/15.
//

#ifndef SENSOR_GPS_H_
#define SENSOR_GPS_H_

//wolf
#include "sensor_base.h"

/*
 * e' la parte hardware dell'albero WOLF
 *
 * parametri intriseci (roba che fa parte del sensore)
 * e estriseci (roba che non ne fa parte (tipo posa iniziale))
 *
 *
 *
 */
class SensorGPS : public SensorBase
{
    //sensor base contiene un puntatore al sensor position e orientation.

    public:
        SensorGPS(StateBlock* _p_ptr, StateBlock* _o_ptr);
        virtual ~SensorGPS();


};


#endif //SENSOR_GPS_H_
