#ifndef FEATURE_CORNER_2D_H_
#define FEATURE_CORNER_2D_H_

//std includes

//Wolf includes
#include "feature_base.h"
//#include "constraint_corner_2D.h"

//class FeatureCorner2D
class FeatureCorner2D : public FeatureBase
{
    public:
        /** \brief Constructor 
         * 
         * constructor
         */
        FeatureCorner2D(const Eigen::Vector4s & _measurement, const Eigen::Matrix4s & _meas_covariance); //TODO: add const WolfScalar& aperture);

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
        WolfScalar getAperture() const; 
        
};
#endif
