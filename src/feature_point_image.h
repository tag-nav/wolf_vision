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
            keypoint_.pt.x = float(measurement_(0));
            keypoint_.pt.y = float(measurement_(1));
        }

        //_known_or_new: known = true; new = false;
        FeaturePointImage(const cv::KeyPoint& _keypoint,
                          const cv::Mat& _descriptor, bool _is_known) :
                FeatureBase(FEATURE_POINT_IMAGE, Eigen::Vector2s::Zero(), Eigen::Matrix2s::Identity()),
                keypoint_(_keypoint),
                descriptor_(_descriptor)
        {
            measurement_(0) = Scalar(_keypoint.pt.x);
            measurement_(1) = Scalar(_keypoint.pt.y);
            is_known_=_is_known;
//            std::cout << "FEATURE TESTING\n";
//            std::cout << "Measurement:\n" << getMeasurement() << std::endl;
//            std::cout << "Measurement Sqrt:\n" << getMeasurementSquareRootInformation() << std::endl;
        }

        FeaturePointImage(const cv::KeyPoint& _keypoint,
                          const cv::Mat& _descriptor, const Eigen::Matrix2s& _meas_covariance) :
                FeatureBase(FEATURE_POINT_IMAGE, Eigen::Vector2s::Zero(), _meas_covariance),
                keypoint_(_keypoint),
                descriptor_(_descriptor)
        {
            measurement_(0) = Scalar(_keypoint.pt.x);
            measurement_(1) = Scalar(_keypoint.pt.y);
        }


        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         */
        virtual ~FeaturePointImage();

        const cv::KeyPoint& getKeypoint() const;
        void setKeypoint(const cv::KeyPoint& _kp)
        {
            keypoint_ = _kp;
        }

        const cv::Mat& getDescriptor() const;
        void setDescriptor(const cv::Mat& _descriptor)
        {
            descriptor_ = _descriptor;
        }

        bool isKnown();
        void setIsKnown(bool _is_known);

        /*Eigen::VectorXs & getMeasurement(){
            measurement_(0) = Scalar(keypoint_.pt.x);
            measurement_(1) = Scalar(keypoint_.pt.y);
            return measurement_;
        }*/

};

inline const cv::KeyPoint& FeaturePointImage::getKeypoint() const
{
    return keypoint_;
}

inline const cv::Mat& FeaturePointImage::getDescriptor() const
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
