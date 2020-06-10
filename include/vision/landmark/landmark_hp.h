#ifndef LANDMARK_HP_H
#define LANDMARK_HP_H

//Wolf includes
#include "core/landmark/landmark_base.h"

// yaml
#include <yaml-cpp/yaml.h>

// Vision utils
#include <vision_utils/vision_utils.h>

namespace wolf {
    
WOLF_PTR_TYPEDEFS(LandmarkHp);

/* Landmark - Homogeneous Point*/
class LandmarkHp : public LandmarkBase
{
    protected:
        cv::Mat cv_descriptor_;


    public:
        LandmarkHp(Eigen::Vector4d _position_homogeneous, SensorBasePtr _sensor_, cv::Mat _2d_descriptor);

        ~LandmarkHp() override;

        const cv::Mat& getCvDescriptor() const;
        void setCvDescriptor(const cv::Mat& _descriptor);

        Eigen::Vector3d point() const;

        YAML::Node saveToYaml() const override;

        /** \brief Creator for Factory<LandmarkBase, YAML::Node>
         * Caution: This creator does not set the landmark's sensor.
         * These need to be set afterwards.
         */
        static LandmarkBasePtr create(const YAML::Node& _node);
};


inline const cv::Mat& LandmarkHp::getCvDescriptor() const
{
    return cv_descriptor_;
}


inline void LandmarkHp::setCvDescriptor(const cv::Mat& _descriptor)
{
    cv_descriptor_ = _descriptor;
}

} // namespace wolf

#endif // LANDMARK_AHP_H
