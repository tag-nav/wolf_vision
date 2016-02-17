#ifndef SENSOR_GPS_H_
#define SENSOR_GPS_H_

/*
 *  NB qui non puoi includere la mia libreria sensor base!
 *  perche in constaint vado a includere questo file, e cio fa in modo che wolf non sia
 *  compilabile senza la mia libreria
 */

//wolf
#include "sensor_base.h"

class SensorGPS : public SensorBase
{
//    TODO parameters needed:
//        * INTRINSIC PARAM:
//                bias                  [ ok ]
//        * EXTRINSIC PARAM:
//                GPS init              [  ]
//                gps mounting point on board in  the vehicle   [ ok ]

protected:

    StateBlock* init_vehicle_p_ptr_; //position of the vehicle where the experiment starts in ecef
    StateBlock* init_vehicle_o_ptr_; //orientation of the vehicle where the experiment starts in ecef

public:

                //pointer to sensor position, orientation, bias, init vehicle position and orientation
    SensorGPS(StateBlock *_p_ptr, StateBlock *_o_ptr, StateBlock* _bias_ptr, StateBlock* _init_vehicle_position_ptr, StateBlock* _init_vehicle_orientation_ptr);

    StateBlock *getInitVehiclePPtr() const;

    StateBlock *getInitVehicleOPtr() const;

    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~SensorGPS();

};


#endif //SENSOR_GPS_H_
