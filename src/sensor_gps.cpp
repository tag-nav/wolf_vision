
#include "sensor_gps.h"


SensorGPS::SensorGPS(StateBlock* _p_ptr, StateBlock* _o_ptr, StateBlock* _bias_ptr) :
                //gps sensor position, orientation and intrinsic param (only bias for now)
        SensorBase(GPS_RAW, _p_ptr, _o_ptr, _bias_ptr, 0)
{
    //std::cout << "SensorGPS constructor... id: " << nodeId() << std::endl;
    //TODO add the 2 new extra extrinsic

}

// TODO Check carefully this destructor!
SensorGPS::~SensorGPS()
{
//    std::cout << "deleting SensorGPS NAV data " << nodeId() << std::endl;
    for (std::list<SatelliteNavData>::iterator it = nav_data_.begin(); it != nav_data_.end() ; ++it)
    {
        std::cout << "---deleting NAV data " << it->getSatId() << std::endl;
        nav_data_.erase(it);
    }
}

void SensorGPS::addNavData(const std::string &_sat_id, const TimeStamp &_timestamp, const WolfScalar &_pseudorange,
                           const WolfScalar &_param1, const WolfScalar &_param2)
{
    SatelliteNavData satellite(_sat_id, _timestamp, _pseudorange, _param1, _param2);

    bool new_satellite = true;

    for (std::list<SatelliteNavData>::iterator it = nav_data_.begin(); it != nav_data_.end() ; ++it)
    {
        if(strcmp(it->getSatId().c_str(), _sat_id.c_str()) == 0)
        {
            *it = satellite; //TOdO sicuro?
            new_satellite = false;
        }
    }

    if(new_satellite)
        nav_data_.push_back(satellite);
}
