#ifndef LANDMARK_AHP_H
#define LANDMARK_AHP_H

//Wolf includes
#include "base/landmark/landmark_base.h"

// yaml
#include <yaml-cpp/yaml.h>

// Vision utils
#include <vision_utils/vision_utils.h>

namespace wolf {
    
WOLF_PTR_TYPEDEFS(LandmarkAHP);

/* Landmark - Anchored Homogeneous Point*/
class LandmarkAHP : public LandmarkBase
{
    protected:
        cv::Mat cv_descriptor_;
        FrameBasePtr anchor_frame_;
        SensorBasePtr anchor_sensor_;

    public:
        LandmarkAHP(Eigen::Vector4s _position_homogeneous, FrameBasePtr _anchor_frame, SensorBasePtr _anchor_sensor, cv::Mat _2D_descriptor);

        virtual ~LandmarkAHP();

        const cv::Mat& getCvDescriptor() const;
        void setCvDescriptor(const cv::Mat& _descriptor);

        const FrameBasePtr  getAnchorFrame () const;
        const SensorBasePtr getAnchorSensor() const;

        void setAnchorFrame  (FrameBasePtr  _anchor_frame );
        void setAnchorSensor (SensorBasePtr _anchor_sensor);
        void setAnchor       (FrameBasePtr  _anchor_frame , SensorBasePtr _anchor_sensor);
        Eigen::Vector3s getPointInAnchorSensor() const;
        Eigen::Vector3s point() const;

        YAML::Node saveToYaml() const;

        /** \brief Creator for Factory<LandmarkBase, YAML::Node>
         * Caution: This creator does not set the landmark's anchor frame and sensor.
         * These need to be set afterwards.
         */
        static LandmarkBasePtr create(const YAML::Node& _node);
};

inline const cv::Mat& LandmarkAHP::getCvDescriptor() const
{
    return cv_descriptor_;
}

inline void LandmarkAHP::setCvDescriptor(const cv::Mat& _descriptor)
{
    cv_descriptor_ = _descriptor;
}

inline const FrameBasePtr LandmarkAHP::getAnchorFrame() const
{
    return anchor_frame_;
}

inline void LandmarkAHP::setAnchorFrame(FrameBasePtr _anchor_frame)
{
    anchor_frame_ = _anchor_frame;
}

inline const SensorBasePtr LandmarkAHP::getAnchorSensor() const
{
    return anchor_sensor_;
}

inline void LandmarkAHP::setAnchorSensor(SensorBasePtr _anchor_sensor)
{
    anchor_sensor_ = _anchor_sensor;
}

inline void LandmarkAHP::setAnchor(FrameBasePtr _anchor_frame, SensorBasePtr _anchor_sensor)
{
    anchor_frame_  = _anchor_frame;
    anchor_sensor_ = _anchor_sensor;
}

} // namespace wolf

#endif // LANDMARK_AHP_H
