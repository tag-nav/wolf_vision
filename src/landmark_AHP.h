#ifndef LANDMARK_AHP_H
#define LANDMARK_AHP_H

//Wolf includes
#include "landmark_base.h"

//OpenCV includes
#include "opencv2/features2d/features2d.hpp"

namespace wolf {

/* Landmark - Anchored Homogeneous Point*/
class LandmarkAHP : public LandmarkBase
{
    protected:
        cv::Mat descriptor_;
        FrameBase* anchor_frame_;

    public:
        LandmarkAHP(Eigen::Vector4s _position, FrameBase* _frame, cv::Mat _2D_descriptor);

        virtual ~LandmarkAHP();

        const cv::Mat& getDescriptor() const;
        void setDescriptor(const cv::Mat& _descriptor)
        {
            descriptor_ = _descriptor;
        }

        const FrameBase* getAnchorFrame() const;
};

inline const cv::Mat& LandmarkAHP::getDescriptor() const
{
    return descriptor_;
}

inline const FrameBase* LandmarkAHP::getAnchorFrame() const
{
    return anchor_frame_;
}

} // namespace wolf

#endif // LANDMARK_AHP_H
