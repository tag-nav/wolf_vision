#include "capture_gps.h"

//this include must stay in the .cpp to avoid circular dependencies of headers
//http://stackoverflow.com/questions/4816698/avoiding-circular-dependencies-of-header-files
#include "constraint_gps_pseudorange.h"

CaptureGPS::CaptureGPS(const TimeStamp &_ts, SensorBase *_sensor_ptr, std::vector<ObsData>& _raw_data) :
        CaptureBase(_ts, _sensor_ptr),
        raw_data_(_raw_data)
{
    //std::cout << "CaptureGPS constructor.\t\tdata.size=" << raw_data_.size() << std::endl;
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
        // TODO attention
        // TODO attention
        // TODO attention
//        in the other examples is in this way (with getFeatureListPtr()->front()) but in this way it adds every time the same  constraint.
//        addFeature(new FeatureGPSPseudorange(raw_data_[i]));
//        getFeatureListPtr()->front()->addConstraintFrom(new ConstraintGPSPseudorange(getFeatureListPtr()->front()));

        //in this way no, but i don't know if it's correct in wolf
        FeatureBase* ftr_ptr = new FeatureGPSPseudorange(raw_data_[i]);

        addFeature(ftr_ptr);
        getFeatureListPtr()->front()->addConstraintFrom(new ConstraintGPSPseudorange(ftr_ptr));

        // TODO attention
        // TODO attention
        // TODO attention
    }

    std::cout << "CaptureGPS::processCapture()... capture processed" << std::endl;
}

/*
 * Dummy implementation of the method, only because it's pure virtual
 */
Eigen::VectorXs CaptureGPS::computePrior(const TimeStamp &_now) const
{
    return Eigen::Vector3s(0, 0, 0);
}

