#include <capture_GPS.h>

namespace wolf {

CaptureGPS::CaptureGPS(const TimeStamp &_ts, SensorBasePtr _sensor_ptr, rawgpsutils::SatellitesObs &_obs) :
        CaptureBase("GPS", _ts, _sensor_ptr),
        obs_(_obs)
{
    //
//    std::cout << "CaptureGPS constructor.\t\tReceived " << obs_.measurements_.size() << " meas" << std::endl;
//    std::cout << obs_.toString() << std::endl;
}


CaptureGPS::~CaptureGPS()
{
    //std::cout << "deleting CaptureGPS " << id() << std::endl;
}

rawgpsutils::SatellitesObs &CaptureGPS::getData()
{
    return obs_;
}


} //namespace wolf
