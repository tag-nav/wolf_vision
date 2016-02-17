//
// Created by ptirindelli on 16/12/15.
//

#include "processor_gps.h"


ProcessorGPS::ProcessorGPS() :
        //sensor_gps_ptr_((SensorGPS*)(upperNodePtr())), //TODO here there's a crash. Look at what they'll do in processorLaser and modify as conseguence
        capture_gps_ptr_(nullptr)
{
    std::cout << "ProcessorGPS constructor" << std::endl;
}

ProcessorGPS::~ProcessorGPS()
{

}

/*
 * Extract features from CaptureGPS and create FeatureGPS
 */
void ProcessorGPS::extractFeatures(CaptureBase *_capture_ptr)
{
    //TODO add assert with dynamic_cast when it will be ready
    capture_gps_ptr_ = (CaptureGPS*)(_capture_ptr);

    std::cout << "Extracting gps features..." << std::endl;

    rawgpsutils::SatellitesObs obs = capture_gps_ptr_->getData();


    //TODO check that the cycle is good (it uses getRawData.size())
    for(unsigned int i = 0; i < obs.measurements_.size(); ++i)
    {
        Eigen::Vector3s sat_pos = obs.measurements_[i].sat_position_;
        WolfScalar pr = obs.measurements_[i].pseudorange_;

        capture_gps_ptr_->addFeature(new FeatureGPSPseudorange(sat_pos, pr));
    }

    //std::cout << "gps features extracted" << std::endl;
}

void ProcessorGPS::establishConstraints(CaptureBase *_capture_ptr)
{
    //TODO add assert with dynamic_cast when it will be ready
    capture_gps_ptr_ = (CaptureGPS*)(_capture_ptr);

    std::cout << "Establishing constraints to gps features..." << std::endl;

    for(auto i_it = capture_gps_ptr_->getFeatureListPtr()->begin(); i_it != capture_gps_ptr_->getFeatureListPtr()->end(); i_it++)
    {
        capture_gps_ptr_->getFeatureListPtr()->front()->addConstraintFrom( new ConstraintGPSPseudorange2D((*i_it)) );
    }


    //std::cout << "Constraints established" << std::endl;
}


