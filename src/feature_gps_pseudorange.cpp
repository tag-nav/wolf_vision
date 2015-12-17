#include "feature_gps_pseudorange.h"

FeatureGPSPseudorange::FeatureGPSPseudorange(ObsData& _satellite_data) : //TODO const?
        FeatureBase(Eigen::VectorXs::Constant(1,_satellite_data.getPseudorange()), Eigen::MatrixXs::Identity(1,1)*0.1),
        obs_(_satellite_data)
//TODO occhio, ora featureBase contiene il pseudorange (posso toglierlo?)
{
//    std::cout << "FeatureGPSPseudorange() constructor -- " << obs_.toString() << std::endl;

    //TODO calculate the satellite position when the message was sent from satellite
    obs_.calculateSatPosition();
}

FeatureGPSPseudorange::~FeatureGPSPseudorange()
{

}

const ObsData *FeatureGPSPseudorange::getObs() const
{
    return &obs_;
}