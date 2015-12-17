#include "capture_gps.h"


CaptureGPS::CaptureGPS(const TimeStamp &_ts, SensorBase *_sensor_ptr, std::vector<ObsData>& _raw_data) :
        CaptureBase(_ts, _sensor_ptr),
        raw_data_(_raw_data)
{
    std::cout << "CaptureGPS constructor.\t\tdata.size=" << raw_data_.size() << std::endl;
}


CaptureGPS::~CaptureGPS()
{
    //std::cout << "deleting CaptureGPS " << nodeId() << std::endl;
}

std::vector<ObsData> &CaptureGPS::getRawData()
{
    return raw_data_;
}


/*
 * Dummy implementation of the method, only because it's pure virtual
 */
Eigen::VectorXs CaptureGPS::computeFramePose(const TimeStamp &_now) const
{
    return Eigen::Vector3s(0, 0, 0);
}
