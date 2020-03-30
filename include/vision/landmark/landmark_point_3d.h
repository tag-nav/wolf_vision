#ifndef LANDMARK_POINT_3d_H
#define LANDMARK_POINT_3d_H

//Wolf includes
#include "core/landmark/landmark_base.h"

// Vision Utils includes
#include <vision_utils/vision_utils.h>

namespace wolf {

WOLF_PTR_TYPEDEFS(LandmarkPoint3d);
    
//class    
class LandmarkPoint3d : public LandmarkBase
{
    protected:
        cv::Mat descriptor_;
        Eigen::Vector3d position_;
    public:
        LandmarkPoint3d(Eigen::Vector3d _position, cv::Mat _2d_descriptor);

        virtual ~LandmarkPoint3d();

        const Eigen::Vector3d point() const;

        const cv::Mat& getDescriptor() const;
        void setDescriptor(const cv::Mat& _descriptor);
};

inline const Eigen::Vector3d LandmarkPoint3d::point() const
{
    return getP()->getState();
}

inline const cv::Mat& LandmarkPoint3d::getDescriptor() const
{
    return descriptor_;
}

inline void LandmarkPoint3d::setDescriptor(const cv::Mat& _descriptor)
{
    descriptor_ = _descriptor;
}

} // namespace wolf

#endif // LANDMARK_POINT_3d_H
