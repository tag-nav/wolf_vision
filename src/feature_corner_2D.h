#ifndef FEATURE_CORNER_2D_H_
#define FEATURE_CORNER_2D_H_

//Wolf includes
#include "feature_base.h"

//std includes


namespace wolf {
    
//forward declaration to typedef class pointers
class FeatureCorner2D;
typedef std::shared_ptr<FeatureCorner2D> FeatureCorner2DPtr;
typedef std::shared_ptr<const FeatureCorner2D> FeatureCorner2DConstPtr;
typedef std::weak_ptr<FeatureCorner2D> FeatureCorner2DWPtr;

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
