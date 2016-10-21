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
//#include "sensor_factory.h"
//#include "factory.h"

// std

namespace wolf {

struct IntrinsicsGPS : public IntrinsicsBase
{
        // add GPS parameters here
};

class SensorGPS : public SensorBase
{
    public:
        typedef std::shared_ptr<SensorGPS> Ptr;
        typedef std::weak_ptr<SensorGPS> WPtr;

protected:

    StateBlockPtr map_p_ptr_; //position of the vehicle where the experiment starts in ecef
    StateBlockPtr map_o_ptr_; //orientation of the vehicle where the experiment starts in ecef

public:
    //pointer to sensor position, orientation, bias, init vehicle position and orientation
    SensorGPS(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, StateBlockPtr _bias_ptr, StateBlockPtr _map_position_ptr, StateBlockPtr _map_orientation_ptr);

    StateBlockPtr getMapPPtr() const;

    StateBlockPtr getMapOPtr() const;

    virtual ~SensorGPS();

    /** \brief Adds all StateBlocks of the frame to the Problem list of new StateBlocks
     **/
    virtual void registerNewStateBlocks();

public:
    static SensorBasePtr create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_p, const IntrinsicsBasePtr _intrinsics);


};

} // namespace wolf

#endif //SENSOR_GPS_H_
