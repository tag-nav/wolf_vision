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

struct IntrinsicsGPS : public IntrinsicsBase
{
        // add GPS parameters here
};

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

    /** \brief Adds all StateBlocks of the frame to the Problem list of new StateBlocks
     **/
    virtual void registerNewStateBlocks();

public:
    static SensorBase* create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_p, const IntrinsicsBase* _intrinsics);


};

} // namespace wolf

#endif //SENSOR_GPS_H_
