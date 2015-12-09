//
// Created by ptirindelli on 3/12/15.
//

#ifndef FEATURE_GPS_PSEUDORANGE_H_
#define FEATURE_GPS_PSEUDORANGE_H_

//TODO indent 1 tab more all the class content

//std includes

//Wolf includes
#include "feature_base.h"

class FeatureGPSPseudorange : public FeatureBase
{
protected:
    std::string satId_;
    Eigen::Vector3s pose_;
//TODO dove metto pseudorange?
    // pseudoranges goes to feature_base's measurement_
    // TODO manage covariance

public:

    FeatureGPSPseudorange(float _satellite_data);

    //virtual ~FeatureGPSPseudorange();



};


#endif //FEATURE_GPS_PSEUDORANGE_H_
