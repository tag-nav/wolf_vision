#ifndef FEATURE_APRILTAG_H_
#define FEATURE_APRILTAG_H_

//Wolf includes
#include "feature_base.h"

//std includes

//external library incudes
#include "apriltag.h"
#include "common/zarray.h"

namespace wolf {
    
WOLF_PTR_TYPEDEFS(FeatureApriltag);

//class FeatureApriltag
class FeatureApriltag : public FeatureBase
{
    public:

        FeatureApriltag(const Eigen::Vector7s & _measurement, const Eigen::Matrix6s & _meas_covariance, const apriltag_detection_t & det);
        virtual ~FeatureApriltag();
        
        /** \brief Returns tag id
         * 
         * Returns tag id
         * 
         **/
        Scalar getTagId() const; 

    private:
    const apriltag_detection_t *det;
        
};

} // namespace wolf

#endif
