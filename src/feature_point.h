#ifndef FEATURE_IMAGE_H
#define FEATURE_IMAGE_H


//Wolf includes
#include "feature_base.h"

//std includes
//

/**
 *
 * Test for the feature point
 *
 **/

//class FeaturePoint
class FeaturePoint : public FeatureBase
{
    protected:

        Eigen::Vector2s measurement_;
        std::vector<float> descriptor_;

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
        FeaturePoint(const Eigen::Vector2s & _measurement, const std::vector<float> & _descriptor);

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

        virtual Eigen::Vector2s getMeasurement();

        virtual std::vector<float> getDescriptor();

};

#endif // FEATURE_IMAGE_H
