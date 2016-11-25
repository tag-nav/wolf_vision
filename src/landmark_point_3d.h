#ifndef LANDMARK_POINT_3D_H
#define LANDMARK_POINT_3D_H


//Wolf includes
#include "landmark_base.h"

//OpenCV includes
#include "opencv2/features2d/features2d.hpp"

// Std includes

namespace wolf {

//forward declaration to typedef class pointers
class LandmarkPoint3D;
typedef std::shared_ptr<LandmarkPoint3D> LandmarkPoint3DPtr;
typedef std::shared_ptr<const LandmarkPoint3D> LandmarkPoint3DConstPtr;
typedef std::weak_ptr<LandmarkPoint3D> LandmarkPoint3DWPtr;
    
//class    
class LandmarkPoint3D : public LandmarkBase
{
    protected:
        cv::Mat descriptor_;
        Eigen::Vector3s position_;
    public:
        LandmarkPoint3D(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, Eigen::Vector3s _position, cv::Mat _2D_descriptor);

        virtual ~LandmarkPoint3D();

        const Eigen::Vector3s& getPosition() const;
        void setPosition(const Eigen::Vector3s & _pos)
        {
            position_ = _pos;
        }

        const cv::Mat& getDescriptor() const;
        void setDescriptor(const cv::Mat& _descriptor)
        {
            descriptor_ = _descriptor;
        }
};

inline const Eigen::Vector3s& LandmarkPoint3D::getPosition() const
{
    return position_;
}

inline const cv::Mat& LandmarkPoint3D::getDescriptor() const
{
    return descriptor_;
}

} // namespace wolf

#endif // LANDMARK_POINT_3D_H
