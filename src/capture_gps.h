#ifndef CAPTURE_GPS_H_
#define CAPTURE_GPS_H_


// Wolf includes
#include "raw_gps_utils/satellites_obs.h"
#include "capture_base.h"

namespace wolf {

class CaptureGPS : public CaptureBase
{
    public:
        typedef std::shared_ptr<CaptureGPS> Ptr;

protected:
    rawgpsutils::SatellitesObs obs_;

public:

    CaptureGPS(const TimeStamp &_ts, SensorBasePtr _sensor_ptr, rawgpsutils::SatellitesObs &_obs);
    virtual ~CaptureGPS();

    rawgpsutils::SatellitesObs &getData();



};

} // namespace wolf

#endif //CAPTURE_GPS_H_
