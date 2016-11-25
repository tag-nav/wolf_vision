#ifndef FEATURE_IMAGE_H
#define FEATURE_IMAGE_H


//Wolf includes
#include "feature_base.h"

//OpenCV includes
#include "opencv2/features2d/features2d.hpp"


namespace wolf {

WOLF_PTR_TYPEDEFS(FeaturePointImage);
    
//class FeaturePointImage
class FeaturePointImage : public FeatureBase
{
    private:
        cv::KeyPoint keypoint_; ///< Warning: every write operation to this member needs to write measurement_. See setKeypoint() as an example.
        cv::Mat descriptor_;
        bool is_known_;
        Scalar score_;
        cv::Rect tracker_roi_;

    public:

        /// Constructor from Eigen measured pixel
        FeaturePointImage(const Eigen::Vector2s & _measured_pixel,
                          const cv::Mat& _descriptor,
                          const Eigen::Matrix2s& _meas_covariance) :
                FeatureBase("POINT IMAGE", _measured_pixel, _meas_covariance),
                keypoint_(_measured_pixel(0), _measured_pixel(1), 1), // Size 1 is a dummy value
                descriptor_( _descriptor),
                is_known_(false),
                score_(0)
        {
            keypoint_.pt.x = measurement_(0);
            keypoint_.pt.y = measurement_(1);
        }

        /// Constructor from OpenCV measured keypoint
        FeaturePointImage(const cv::KeyPoint& _keypoint,
                          const cv::Mat& _descriptor,
                          const Eigen::Matrix2s& _meas_covariance) :
                FeatureBase("POINT IMAGE", Eigen::Vector2s::Zero(), _meas_covariance),
                keypoint_(_keypoint),
                descriptor_(_descriptor),
                is_known_(false),
                score_(0)
        {
            measurement_(0) = Scalar(_keypoint.pt.x);
            measurement_(1) = Scalar(_keypoint.pt.y);
        }

       virtual ~FeaturePointImage();

        const cv::KeyPoint& getKeypoint() const;
        void setKeypoint(const cv::KeyPoint& _kp);

        const cv::Mat& getDescriptor() const;
        void setDescriptor(const cv::Mat& _descriptor);

        bool isKnown();
        void setIsKnown(bool _is_known);

        /*Eigen::VectorXs & getMeasurement(){
            measurement_(0) = Scalar(keypoint_.pt.x);
            measurement_(1) = Scalar(keypoint_.pt.y);
            return measurement_;
        }*/

        const cv::Rect& getTrackerRoi() const;
        void setTrackerRoi(const cv::Rect& tracker_roi);

        Scalar getScore() const
        {
            return score_;
        }

        void setScore(Scalar score)
        {
            score_ = score;
        }
};

inline const cv::KeyPoint& FeaturePointImage::getKeypoint() const
{
    return keypoint_;
}

inline void FeaturePointImage::setKeypoint(const cv::KeyPoint& _kp)
{
    keypoint_ = _kp;
    measurement_(0) = _kp.pt.x;
    measurement_(1) = _kp.pt.y;
}

inline const cv::Mat& FeaturePointImage::getDescriptor() const
{
    return descriptor_;
}

inline void FeaturePointImage::setDescriptor(const cv::Mat& _descriptor)
{
    descriptor_ = _descriptor;
}

inline bool FeaturePointImage::isKnown()
{
    return is_known_;
}

inline void FeaturePointImage::setIsKnown(bool _is_known)
{
    is_known_ = _is_known;
}

inline const cv::Rect& FeaturePointImage::getTrackerRoi() const
{
    return tracker_roi_;
}

inline void FeaturePointImage::setTrackerRoi(const cv::Rect& tracker_roi)
{
    tracker_roi_ = tracker_roi;
}

} // namespace wolf

#endif // FEATURE_IMAGE_H
