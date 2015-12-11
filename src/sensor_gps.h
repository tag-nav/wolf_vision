//
// Created by ptirindelli on 3/12/15.
//

//TODO indentation: add a tab at everything inside the class: protected and public must have 1 tab


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
 * TODO qua dentro va la nav Data, perche' in teoria cambiano a slow rate
 * ((previously)qua dentro va il nav file, perche' e' fisso e non cambia con il tempo)
 *
 */
class SensorGPS : public SensorBase
{
//todo nav data
public:
    //pointer to sensor position and orientation.
    SensorGPS(StateBlock *_p_ptr, StateBlock *_o_ptr, StateBlock* _bias_ptr);

    virtual ~SensorGPS();



};


#endif //SENSOR_GPS_H_
