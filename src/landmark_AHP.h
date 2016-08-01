#ifndef LANDMARK_AHP_H
#define LANDMARK_AHP_H

//Wolf includes
#include "landmark_base.h"

// yaml
#include <yaml-cpp/yaml.h>

//OpenCV includes
#include "opencv2/features2d/features2d.hpp"

namespace wolf {

/* Landmark - Anchored Homogeneous Point*/
class LandmarkAHP : public LandmarkBase
{
    protected:
        cv::Mat cv_descriptor_;
        FrameBase* anchor_frame_;
        SensorBase* anchor_sensor_;

    public:
        LandmarkAHP(Eigen::Vector4s _position_homogeneous, FrameBase* _anchor_frame, SensorBase* _anchor_sensor, cv::Mat _2D_descriptor);

        virtual ~LandmarkAHP();

        const cv::Mat& getCvDescriptor() const;
        void setCvDescriptor(const cv::Mat& _descriptor);

        const FrameBase*  getAnchorFrame () const;
        const SensorBase* getAnchorSensor() const;

        void setAnchorFrame  (FrameBase*  _anchor_frame );
        void setAnchorSensor (SensorBase* _anchor_sensor);
        void setAnchor       (FrameBase*  _anchor_frame , SensorBase* _anchor_sensor);

        YAML::Node saveToYaml() const;

        /** \brief Creator for Factory<LandmarkBase, YAML::Node>
         * Caution: This creator does not set the landmark's anchor frame and sensor.
         * These need to be set afterwards.
         */
        static LandmarkBase* create(const YAML::Node& _node);
};

inline const cv::Mat& LandmarkAHP::getCvDescriptor() const
{
    return cv_descriptor_;
}

inline void LandmarkAHP::setCvDescriptor(const cv::Mat& _descriptor)
{
    cv_descriptor_ = _descriptor;
}

inline const FrameBase* LandmarkAHP::getAnchorFrame() const
{
    return anchor_frame_;
}

inline void LandmarkAHP::setAnchorFrame(FrameBase* _anchor_frame)
{
    anchor_frame_ = _anchor_frame;
}

inline const SensorBase* LandmarkAHP::getAnchorSensor() const
{
    return anchor_sensor_;
}

inline void LandmarkAHP::setAnchorSensor(SensorBase* _anchor_sensor)
{
    anchor_sensor_ = _anchor_sensor;
}

inline void LandmarkAHP::setAnchor(FrameBase* _anchor_frame, SensorBase* _anchor_sensor)
{
    anchor_frame_  = _anchor_frame;
    anchor_sensor_ = _anchor_sensor;
}

} // namespace wolf

#endif // LANDMARK_AHP_H
