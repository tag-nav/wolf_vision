#ifndef FEATURE_CORNER_2D_H_
#define FEATURE_CORNER_2D_H_

//Wolf includes
#include "feature_base.h"

//std includes


namespace wolf {

//class FeatureCorner2D
class FeatureCorner2D : public FeatureBase
{
    public:
        /** \brief Constructor 
         * 
         * constructor
         */
        FeatureCorner2D(const Eigen::Vector4s & _measurement, const Eigen::Matrix4s & _meas_covariance); //TODO: add const Scalar& aperture);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         */        
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
