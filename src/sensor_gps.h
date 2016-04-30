#ifndef SENSOR_GPS_H_
#define SENSOR_GPS_H_

/* WARNING: from here you cannot include gps_scan_utils
 * because is included in constraintGPS, and constraintGPS is included in wolf.h (or some other wolf file)
 * Otherwise wolf will not build without my library installed
 *
 * --- MAYBE IS NO MORE TRUE, AFTER THE INCLUDES FIX!! ---
 */

//wolf
#include "sensor_base.h"
#include "sensor_factory.h"

// std

namespace wolf {

class SensorGPS : public SensorBase
{
protected:

    StateBlock* map_p_ptr_; //position of the vehicle where the experiment starts in ecef
    StateBlock* map_o_ptr_; //orientation of the vehicle where the experiment starts in ecef

public:
                //pointer to sensor position, orientation, bias, init vehicle position and orientation
    SensorGPS(StateBlock *_p_ptr, StateBlock *_o_ptr, StateBlock* _bias_ptr, StateBlock* _map_position_ptr, StateBlock* _map_orientation_ptr);

    StateBlock *getMapPPtr() const;

    StateBlock *getMapOPtr() const;

    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     **/
    virtual ~SensorGPS();

    /** \brief Adds all stateBlocks of the frame to the wolfProblem list of new stateBlocks
     **/
    virtual void registerNewStateBlocks();

};

//// Define the factory method and register it in the SensorFactory
//namespace
//{
//SensorBase* createGPS(std::string& _name, std::string _params_filename = "")
//{
//    SensorBase* sen = new SensorGPS(nullptr, nullptr, nullptr, nullptr, nullptr);
//    sen->setName(_name);
//    return sen;
//}
//const bool registered_gps = SensorFactory::get()->registerSensor("GPS_RAW", createGPS);
//}



} // namespace wolf

#endif //SENSOR_GPS_H_
