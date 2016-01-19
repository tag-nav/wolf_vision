
#include "sensor_gps.h"


SensorGPS::SensorGPS(StateBlock* _p_ptr, //GPS sensor position with respect to the car frame (base frame)
                     StateBlock* _o_ptr, //GPS sensor orientation with respect to the car frame (base frame)
                     StateBlock* _bias_ptr,  //GPS sensor time bias
                     StateBlock* _init_vehicle_position_ptr, //initial position of vehicle's frame with respect to starting point frame
                     StateBlock* _init_vehicle_orientation_ptr) //initial orientation of vehicle's frame with respect to the starting point frame
        :
        SensorBase(GPS_RAW, _p_ptr, _o_ptr, _bias_ptr, 0),
        init_vehicle_position_ptr_(_init_vehicle_position_ptr),
        init_vehicle_orientation_ptr_(_init_vehicle_orientation_ptr)
{
    //std::cout << "SensorGPS constructor... id: " << nodeId() << std::endl;
}

// TODO Check carefully this destructor!
SensorGPS::~SensorGPS()
{
//    std::cout << "deleting SensorGPS NAV data " << nodeId() << std::endl;
//    for (std::list<rawgpsutils::NavData>::iterator it = nav_data_.begin(); it != nav_data_.end() ; ++it)
//    {
//        std::cout << "---deleting NAV data " << it->getSatId() << std::endl;
//        nav_data_.erase(it);
//    }
}

StateBlock *SensorGPS::getInitVehiclePPtr() const
{
    return init_vehicle_position_ptr_;
}

StateBlock *SensorGPS::getInitVehicleOPtr() const
{
    return init_vehicle_orientation_ptr_;
}


//
//  ¡¡¡¡NO NAV DATA HERE!!!!
//
//void SensorGPS::addNavData(const std::string &_sat_id, const TimeStamp &_timestamp, const WolfScalar &_pseudorange,
//                           const WolfScalar &_param1, const WolfScalar &_param2)
//{
//    rawgpsutils::NavData satellite(_sat_id, _timestamp, _pseudorange, _param1, _param2);
//
//    bool new_satellite = true;
//
//    for (std::list<rawgpsutils::NavData>::iterator it = nav_data_.begin(); it != nav_data_.end() ; ++it)
//    {
//        if(strcmp(it->getSatId().c_str(), _sat_id.c_str()) == 0)
//        {
//            *it = satellite; //TOdO sicuro?
//            new_satellite = false;
//        }
//    }
//
//    if(new_satellite)
//        nav_data_.push_back(satellite);
//}


