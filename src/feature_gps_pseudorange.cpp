#include "feature_gps_pseudorange.h"

using namespace std;

FeatureGPSPseudorange::FeatureGPSPseudorange(const WolfScalar& _satellite_data) :
        FeatureBase(Eigen::VectorXs::Constant(1,_satellite_data), Eigen::MatrixXs::Identity(1,1)*0.1)
{
    satId_ = "satId_n";//TODO metti nome vero
    cout << "#FEATURE#   Satellite: " << _satellite_data << endl;

}

FeatureGPSPseudorange::~FeatureGPSPseudorange()
{

}
