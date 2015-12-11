#include "capture_gps.h"

CaptureGPS::CaptureGPS(const TimeStamp &_ts, SensorBase *_sensor_ptr, const Eigen::VectorXs &_raw_data) :
        CaptureBase(_ts, _sensor_ptr),
        raw_data_(_raw_data)
{
    std::cout << "CaptureGPS constructor.\t\tdata.size=" << raw_data_.size() << std::endl;
}


CaptureGPS::~CaptureGPS()
{
    //std::cout << "deleting CaptureGPS " << nodeId() << std::endl;
}

/** \brief Process a gps capture
 *
 * Process a gps and create feature and constraint for each satellite
 *
 **/
void CaptureGPS::processCapture()
{
    std::cout << "CaptureGPS::processCapture()... processing capture" << std::endl;

    // EXTRACT AND ADD FEATURES AND CONSTRAINTS
    for(unsigned int i = 0; i < raw_data_.size(); ++i)
    {
        addFeature(new FeatureGPSPseudorange(raw_data_[i]));
        getFeatureListPtr()->front()->addConstraintFrom(new ConstraintGPSPseudorange(getFeatureListPtr()->front()));
    }

    std::cout << "CaptureGPS::processCapture()... capture processed" << std::endl;
}

// TODO what does this function do? Do I really need it? (i have to implement it because is pure virtual, but now in my implementation it's meaningless)
Eigen::VectorXs CaptureGPS::computePrior(const TimeStamp &_now) const
{
    return Eigen::Vector3s(1, 2, 3); //TODO implementata a caso, solo perche' e' virtual pura e altrimenti non compila
}

