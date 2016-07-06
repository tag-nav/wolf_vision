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
        StateBlock* position;
        StateBlock* anchorRobot_;

    public:
        LandmarkAHP(StateBlock* _p_ptr, StateBlock* _o_ptr, cv::Mat _2D_descriptor, Eigen::Vector4s _position, FrameBase* _frame);

        virtual ~LandmarkAHP();

        const cv::Mat& getDescriptor() const;
        void setDescriptor(const cv::Mat& _descriptor)
        {
            descriptor_ = _descriptor;
        }

};

inline const cv::Mat& LandmarkAHP::getDescriptor() const
{
    return descriptor_;
}

} // namespace wolf

#endif // LANDMARK_AHP_H
