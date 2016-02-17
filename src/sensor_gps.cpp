
#include "sensor_gps.h"


SensorGPS::SensorGPS(StateBlock* _p_ptr, //GPS sensor position with respect to the car frame (base frame)
                     StateBlock* _o_ptr, //GPS sensor orientation with respect to the car frame (base frame)
                     StateBlock* _bias_ptr,  //GPS sensor time bias
                     StateBlock* _init_vehicle_p_ptr, //initial position of vehicle's frame with respect to starting point frame
                     StateBlock* _init_vehicle_o_ptr) //initial orientation of vehicle's frame with respect to the starting point frame
        :
        SensorBase(GPS_RAW, _p_ptr, _o_ptr, _bias_ptr, 0),
        init_vehicle_p_ptr_(_init_vehicle_p_ptr),
        init_vehicle_o_ptr_(_init_vehicle_o_ptr)
{
    //std::cout << "SensorGPS constructor... id: " << nodeId() << std::endl;
}

// TODO Check carefully this destructor!
SensorGPS::~SensorGPS()
{

}

StateBlock *SensorGPS::getInitVehiclePPtr() const
{
    return init_vehicle_p_ptr_;
}

StateBlock *SensorGPS::getInitVehicleOPtr() const
{
    return init_vehicle_o_ptr_;
}
