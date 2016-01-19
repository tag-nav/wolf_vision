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


/* NOTA BENE NB
 * il processor lavora al livello della capture! dalla capture legge il vector di obs,
 * fa tutti i conti usando la mia libreria e poi crea le feature che sono semplicissime,
 * solo pr e sat pos
 */
void ProcessorGPS::extractFeatures(CaptureBase *_capture_ptr)
{
    //TODO add assert with dynamic_cast when it will be ready
    capture_gps_ptr_ = (CaptureGPS*)(_capture_ptr);

    std::cout << "Extracting gps features..." << std::endl;

    //TODO check that the cycle is good (it uses getRawData.size())
    for(unsigned int i = 0; i < capture_gps_ptr_->getRawData().size(); ++i)
    {
        //TODO qui calcola pr e sat position, e crea feature con solo sti dati

        //TODO fa i conti, usando la mia libreria esterna volendo
        capture_gps_ptr_->getRawData()[i].calculateSatPosition();
        Eigen::Vector3s sat_pos = capture_gps_ptr_->getRawData()[i].getSatPosition();
        WolfScalar pr = capture_gps_ptr_->getRawData()[i].getPseudorange();



        capture_gps_ptr_->addFeature(new FeatureGPSPseudorange(sat_pos, pr));
    }

    std::cout << "gps features extracted" << std::endl;
}

void ProcessorGPS::establishConstraints(CaptureBase *_capture_ptr)
{
    //TODO add assert with dynamic_cast when it will be ready
    capture_gps_ptr_ = (CaptureGPS*)(_capture_ptr);

    std::cout << "Establishing constraints to gps features..." << std::endl;

    for(auto i_it = capture_gps_ptr_->getFeatureListPtr()->begin(); i_it != capture_gps_ptr_->getFeatureListPtr()->end(); i_it++)
    {
        capture_gps_ptr_->getFeatureListPtr()->front()->addConstraintFrom( new ConstraintGPSPseudorange((*i_it)) );
    }


    std::cout << "Constraints established" << std::endl;
}


