#ifndef FEATURE_IMAGE_H
#define FEATURE_IMAGE_H


//Wolf includes
#include "feature_base.h"

//OpenCV includes
#include "opencv2/features2d/features2d.hpp"


namespace wolf {

/**
 *
 * Test for the feature point
 *
 **/

//class FeaturePointImage
class FeaturePointImage : public FeatureBase
{
    protected:

        cv::KeyPoint keypoint_;
        cv::Mat descriptor_;
        bool is_known_;

    public:
        FeaturePointImage(const Eigen::Vector2s & _measurement);

        FeaturePointImage(const Eigen::Vector2s & _measurement, const Eigen::Matrix2s& _meas_covariance) :
                FeatureBase(FEATURE_POINT_IMAGE, _measurement, _meas_covariance)
        {
            keypoint_.pt.x = measurement_(0);
            keypoint_.pt.y = measurement_(1);
        }

        //_known_or_new: known = true; new = false;
        FeaturePointImage(const cv::KeyPoint& _keypoint,
                          const cv::Mat& _descriptor, bool _is_known) :
                FeatureBase(FEATURE_POINT_IMAGE, Eigen::Vector2s::Zero(), Eigen::Matrix2s::Identity()),
                keypoint_(_keypoint),
                descriptor_(_descriptor)
        {
            measurement_(0) = _keypoint.pt.x;
            measurement_(1) = _keypoint.pt.y;
            is_known_=_is_known;
        }


        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         */
        virtual ~FeaturePointImage();

        cv::KeyPoint& getKeypoint();

        cv::Mat& getDescriptor();

        bool isKnown();
        void setIsKnown(bool _is_known);

        Eigen::VectorXs & getMeasurement(){
            measurement_(0) = Scalar(keypoint_.pt.x);
            measurement_(1) = Scalar(keypoint_.pt.y);
            return measurement_;
        }

};

inline cv::KeyPoint& FeaturePointImage::getKeypoint()
{
    return keypoint_;
}

inline cv::Mat& FeaturePointImage::getDescriptor()
{
    return descriptor_;
}

inline bool FeaturePointImage::isKnown()
{
    return is_known_;
}

inline void FeaturePointImage::setIsKnown(bool _is_known)
{
    is_known_ = _is_known;
}

} // namespace wolf

#endif // FEATURE_IMAGE_H
