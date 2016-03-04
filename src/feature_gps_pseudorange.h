
#ifndef FEATURE_GPS_PSEUDORANGE_H_
#define FEATURE_GPS_PSEUDORANGE_H_

//Wolf includes
#include "feature_base.h"

//std includes
#include <iomanip>

// TODO manage covariance

class FeatureGPSPseudorange : public FeatureBase
{
protected:
    Eigen::Vector3s sat_position_;
    WolfScalar pseudorange_;

public:
    FeatureGPSPseudorange(Eigen::Vector3s& _sat_position, WolfScalar _pseudorange );

    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~FeatureGPSPseudorange();

    const Eigen::Vector3s & getSatPosition() const;

    WolfScalar getPseudorange() const;

};


#endif //FEATURE_GPS_PSEUDORANGE_H_
