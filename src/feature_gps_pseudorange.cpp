//
// Created by ptirindelli on 3/12/15.
//

#include "feature_gps_pseudorange.h"

using namespace std;

FeatureGPSPseudorange::FeatureGPSPseudorange(float _satellite_data)
                                : FeatureBase(1) // TODO cosi sto chiamando il costruttore con la dimensione = 1
{
    cout << "#FEATURE#   Satellite: " << _satellite_data << endl;

}

