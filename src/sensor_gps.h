//TODO indentation: add a tab at everything inside the class: protected and public must have 1 tab


#ifndef SENSOR_GPS_H_
#define SENSOR_GPS_H_

//wolf
#include "sensor_base.h"


class SensorGPS : public SensorBase
{
protected:
//    TODO parameters needed:
//        * INTRINSIC PARAM:
//                bias
//        * EXTRINSIC PARAM:
//                nav data
//                gps mounting point

public:
    //pointer to sensor position and orientation.
    SensorGPS(StateBlock *_p_ptr, StateBlock *_o_ptr, StateBlock* _bias_ptr);

    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~SensorGPS();

};


#endif //SENSOR_GPS_H_
