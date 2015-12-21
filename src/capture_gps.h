#ifndef CAPTURE_GPS_H_
#define CAPTURE_GPS_H_

//TODO indentation: add a tab at everything inside the class: protected and public must have 1 tab

// Wolf includes
#include "raw_gps_utils/obs_data.h"
#include "capture_base.h"


class CaptureGPS : public CaptureBase
{

protected:
    std::vector<rawgpsutils::ObsData> raw_data_;

public:
    std::vector<rawgpsutils::ObsData> &getRawData();

    CaptureGPS(const TimeStamp& _ts, SensorBase* _sensor_ptr, std::vector<rawgpsutils::ObsData>& _raw_data);

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



#endif //CAPTURE_GPS_H_
