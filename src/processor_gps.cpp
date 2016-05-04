//
// Created by ptirindelli on 16/12/15.
//

#include "processor_gps.h"
#include "feature_gps_pseudorange.h"
#include "constraint_gps_pseudorange_2D.h"


namespace wolf {

ProcessorGPS::ProcessorGPS() : ProcessorBase(PRC_GPS_RAW),
        //sensor_gps_ptr_((SensorGPS*)(upperNodePtr())), //TODO here there's a crash. Look at what they'll do in processorLaser and modify as consequence
        capture_gps_ptr_(nullptr)
{
    setType("GPS");
    gps_covariance_ = 10;
}

ProcessorGPS::~ProcessorGPS()
{

}

/*
 * Extract feature from the capture and create constraint
 */
void ProcessorGPS::process(CaptureBase* _capture_ptr)
{
    std::cout << "ProcessorGPS::process(GPScapture)" << std::endl;

    //TODO add assert with dynamic_cast when it will be ready
    capture_gps_ptr_ = (CaptureGPS*)(_capture_ptr);

    //std::cout << "Extracting gps features..." << std::endl;
    rawgpsutils::SatellitesObs obs = capture_gps_ptr_->getData();


    for(unsigned int i = 0; i < obs.measurements_.size(); ++i)
    {
        Eigen::Vector3s sat_pos = obs.measurements_[i].sat_position_;
        Scalar pr = obs.measurements_[i].pseudorange_;

        capture_gps_ptr_->addFeature(new FeatureGPSPseudorange(sat_pos, pr, gps_covariance_));
    }
    //std::cout << "gps features extracted" << std::endl;


    //std::cout << "Establishing constraints to gps features..." << std::endl;
    for(auto i_it = capture_gps_ptr_->getFeatureListPtr()->begin(); i_it != capture_gps_ptr_->getFeatureListPtr()->end(); i_it++)
    {
        capture_gps_ptr_->getFeatureListPtr()->front()->addConstraint( new ConstraintGPSPseudorange2D((*i_it)) );
    }
    //std::cout << "Constraints established" << std::endl;
}

ProcessorBase* ProcessorGPS::create(const std::string& _unique_name, const ProcessorParamsBase* _params)
{
    ProcessorGPS* prc_ptr = new ProcessorGPS();
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

} // namespace wolf
