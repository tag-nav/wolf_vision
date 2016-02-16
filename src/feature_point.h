#ifndef FEATURE_IMAGE_H
#define FEATURE_IMAGE_H


//std includes

//Wolf includes
#include "feature_base.h"

/**
 *
 * Test for the feature point
 *
 **/

//class FeaturePoint
class FeaturePoint : public FeatureBase
{
    public:
        /** \brief Constructor
         *
         * constructor
         */
        FeaturePoint(const Eigen::Vector2s & _measurement);

        /** \brief Constructor
         *
         * constructor
         */
        FeaturePoint(const Eigen::Vector2s & _measurement, const Eigen::Matrix2s & _meas_covariance);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         */
        virtual ~FeaturePoint();

};

#endif // FEATURE_IMAGE_H
