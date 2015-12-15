#include "feature_gps_pseudorange.h"

using namespace std;

FeatureGPSPseudorange::FeatureGPSPseudorange(ObsData& _satellite_data) : //TODO const?
        FeatureBase(Eigen::VectorXs::Constant(1,_satellite_data.getPseudorange()), Eigen::MatrixXs::Identity(1,1)*0.1),
        obs_(_satellite_data)
//TODO occhio, ora featureBase contiene il pseudorange
{
    //cout << "FeatureGPSPseudorange() constructor -- " << obs_.toString() << endl;

    //calculate the satellite position when the message was sent from satellite
    obs_.calculateSatPosition();
}

FeatureGPSPseudorange::~FeatureGPSPseudorange()
{

}
