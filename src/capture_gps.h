#ifndef CAPTURE_GPS_H_
#define CAPTURE_GPS_H_


// Wolf includes
#include "iri-algorithms/raw_gps_utils/satellites_obs.h"
#include "capture_base.h"

namespace wolf {

class CaptureGPS : public CaptureBase
{

protected:
    rawgpsutils::SatellitesObs obs_;

public:

    CaptureGPS(const TimeStamp &_ts, SensorBase *_sensor_ptr, rawgpsutils::SatellitesObs &_obs);

    rawgpsutils::SatellitesObs &getData();

    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~CaptureGPS();

    /*
     * Dummy implementation of the method, only because it's pure virtual
     */
    virtual Eigen::VectorXs computeFramePose(const TimeStamp &_now) const;

};

} // namespace wolf

#endif //CAPTURE_GPS_H_
