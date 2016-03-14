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

//        Eigen::Vector2s measurement_;
        cv::KeyPoint keypoint_;
        std::vector<float> descriptor_;

    public:
        FeaturePointImage(const Eigen::Vector2s & _measurement);

        FeaturePointImage(const Eigen::Vector2s& _measurement, const Eigen::Matrix2s& _meas_covariance) :
                FeatureBase(FEAT_POINT_IMAGE, _measurement, _meas_covariance)
        {
            keypoint_.pt.x = measurement_(0);
            keypoint_.pt.y = measurement_(1);
        }

        FeaturePointImage(const cv::KeyPoint _keypoint,
                          const std::vector<float>& _descriptor) :
                FeatureBase(FEAT_POINT_IMAGE, Eigen::MatrixXs::Zero(0, 0), Eigen::MatrixXs::Zero(0, 0)),
                keypoint_(_keypoint),
                descriptor_(_descriptor)
        {
            measurement_(0) = _keypoint.pt.x;
            measurement_(1) = _keypoint.pt.y;
        }


        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         */
        virtual ~FeaturePointImage();

        virtual cv::KeyPoint& getKeypoint();

        virtual std::vector<float>& getDescriptor();

        virtual Eigen::VectorXs & getMeasurement(){
            measurement_(0) = WolfScalar(keypoint_.pt.x);
            measurement_(1) = WolfScalar(keypoint_.pt.y);
            return measurement_;
        }

};

inline cv::KeyPoint& FeaturePointImage::getKeypoint()
{
    return keypoint_;
}

inline std::vector<float>& FeaturePointImage::getDescriptor()
{
    return descriptor_;
}

#endif // FEATURE_IMAGE_H
