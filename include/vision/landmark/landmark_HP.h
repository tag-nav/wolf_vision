#ifndef LANDMARK_AHP_H
#define LANDMARK_AHP_H

//Wolf includes
#include "core/landmark/landmark_base.h"

// yaml
#include <yaml-cpp/yaml.h>

// Vision utils
#include <vision_utils/vision_utils.h>

namespace wolf {
    
WOLF_PTR_TYPEDEFS(LandmarkHP);

/* Landmark - Homogeneous Point*/
class LandmarkHP : public LandmarkBase
{
    protected:
    	SensorBasePtr sensor_;
        cv::Mat cv_descriptor_;


    public:
        LandmarkHP(Eigen::Vector4s _position_homogeneous, SensorBasePtr _sensor_, cv::Mat _2D_descriptor);

        virtual ~LandmarkHP();

        const cv::Mat& getCvDescriptor() const;
        void setCvDescriptor(const cv::Mat& _descriptor);

        void setSensor (SensorBasePtr _anchor_sensor);
        const SensorBasePtr getSensor() const;

        Eigen::Vector3s point() const;

        YAML::Node saveToYaml() const;

        /** \brief Creator for Factory<LandmarkBase, YAML::Node>
         * Caution: This creator does not set the landmark's sensor.
         * These need to be set afterwards.
         */
        static LandmarkBasePtr create(const YAML::Node& _node);
};

inline void LandmarkHP::setSensor(SensorBasePtr _sensor)
{
   sensor_ = _sensor;
}


inline const SensorBasePtr LandmarkHP::getSensor() const
{
    return sensor_;
}


inline const cv::Mat& LandmarkHP::getCvDescriptor() const
{
    return cv_descriptor_;
}


inline void LandmarkHP::setCvDescriptor(const cv::Mat& _descriptor)
{
    cv_descriptor_ = _descriptor;
}

} // namespace wolf

#endif // LANDMARK_AHP_H