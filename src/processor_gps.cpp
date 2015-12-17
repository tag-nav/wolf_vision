//
// Created by ptirindelli on 16/12/15.
//

#include "processor_gps.h"


ProcessorGPS::ProcessorGPS() :
        sensor_gps_ptr_((SensorGPS*)(upperNodePtr())), // Static cast to specific sensor at construction time
        capture_gps_ptr_(nullptr)
{
    std::cout << "ProcessorGPS constructor" << std::endl;
}

ProcessorGPS::~ProcessorGPS()
{

}

void ProcessorGPS::extractFeatures(CaptureBase *_capture_ptr)
{
    //TODO add assert with dynamic_cast when it will be ready
    capture_gps_ptr_ = (CaptureGPS*)(_capture_ptr);

    std::cout << "Extracting gps features..." << std::endl;

    for(unsigned int i = 0; i < capture_gps_ptr_->getRawData().size(); ++i)
    {
        capture_gps_ptr_->addFeature(new FeatureGPSPseudorange(capture_gps_ptr_->getRawData()[i]));
    }

    std::cout << "gps features extracted" << std::endl;
}

void ProcessorGPS::establishConstraints(CaptureBase *_capture_ptr)
{
    //TODO add assert with dynamic_cast when it will be ready
    capture_gps_ptr_ = (CaptureGPS*)(_capture_ptr);

    std::cout << "Establishing constraints to gps features..." << std::endl;

    for(unsigned int i = 0; i < capture_gps_ptr_->getRawData().size(); ++i)
    {
        capture_gps_ptr_->getFeatureListPtr()->front()->addConstraintFrom(new ConstraintGPSPseudorange(capture_gps_ptr_->getFeatureListPtr()->back()));
    }

    std::cout << "Constraints established" << std::endl;
}


