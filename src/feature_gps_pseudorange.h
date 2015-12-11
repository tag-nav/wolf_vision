//
// Created by ptirindelli on 3/12/15.
//

#ifndef FEATURE_GPS_PSEUDORANGE_H_
#define FEATURE_GPS_PSEUDORANGE_H_

//TODO indent 1 tab more all the class content

//std includes

//Wolf includes
#include "feature_base.h"



// TODO manage covariance

class FeatureGPSPseudorange : public FeatureBase
{
protected:
    Eigen::Vector3s pose_;
    std::string satId_;
    //TODO dove metto pseudorange?
    // pseudoranges goes to feature_base's measurement_

    //TODO cosa salvo nelle feature?
    // in teoria qui potrei mettere solo timestamp, satId e pr
    // poi la posizione del satellite al momento timestamp
    // la calcola qualcun'altro, o nol constrAINT o qui con una funz



public:

    // TODO getter generato solo per prova,
    // vedi se serve e in caso spostalo  nel cpp
    // (prima controlla che normalmente si faccia cosi)
    const std::string &getSatId_() const {
        return satId_;
    }


    FeatureGPSPseudorange(const WolfScalar &_satellite_data);

    virtual ~FeatureGPSPseudorange();



};


#endif //FEATURE_GPS_PSEUDORANGE_H_
