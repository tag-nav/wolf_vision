#ifndef FEATURE_CORNER_2D_H_
#define FEATURE_CORNER_2D_H_

//Wolf includes
#include "feature_base.h"

//std includes


namespace wolf {
    
WOLF_PTR_TYPEDEFS(FeatureCorner2D);

//class FeatureCorner2D
class FeatureCorner2D : public FeatureBase
{
    public:

        FeatureCorner2D(const Eigen::Vector4s & _measurement, const Eigen::Matrix4s & _meas_covariance);
        virtual ~FeatureCorner2D();
        
        /** \brief Returns aperture
         * 
         * Returns aperture
         * 
         **/
        Scalar getAperture() const; 
        
};

} // namespace wolf

#endif
