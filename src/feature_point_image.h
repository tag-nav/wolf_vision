#ifndef FEATURE_IMAGE_H
#define FEATURE_IMAGE_H


//Wolf includes
#include "feature_base.h"

//OpenCV includes
#include "opencv2/features2d/features2d.hpp"

/**
 *
 * Test for the feature point
 *
 **/

//class FeaturePointImage
class FeaturePointImage : public FeatureBase
{
    protected:

        Eigen::Vector2s measurement_;
        std::vector<float> descriptor_;
        cv::KeyPoint keypoint_;

    public:
        /** \brief Constructor
         *
         * constructor
         */
        FeaturePointImage(const Eigen::Vector2s & _measurement);

        /** \brief Constructor
         *
         * constructor
         */
        FeaturePointImage(const Eigen::Vector2s & _measurement, const cv::KeyPoint _keypoint,const std::vector<float> & _descriptor);

        /** \brief Constructor
         *
         * constructor
         */
        FeaturePointImage(const Eigen::Vector2s & _measurement, const Eigen::Matrix2s & _meas_covariance);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         */
        virtual ~FeaturePointImage();

        virtual Eigen::Vector2s getMeasurement();

        virtual cv::KeyPoint getKeypoint();

        virtual std::vector<float> getDescriptor();

};

#endif // FEATURE_IMAGE_H
