
#ifndef FEATURE_GPS_PSEUDORANGE_H_
#define FEATURE_GPS_PSEUDORANGE_H_

//TODO indent 1 tab more all the class content

//std includes

//Wolf includes
#include "raw_data_satellite.h"
#include "feature_base.h"

// TODO manage covariance

class FeatureGPSPseudorange : public FeatureBase
{
protected:
    ObsData obs_;

public:

    FeatureGPSPseudorange(ObsData& _satellite_data);

    /** \brief Default destructor (not recommended)
     *
     * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
     *
     **/
    virtual ~FeatureGPSPseudorange();


    const ObsData* getObs() const;
};


#endif //FEATURE_GPS_PSEUDORANGE_H_
