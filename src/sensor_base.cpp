#include "sensor_base.h"

SensorBase::SensorBase(const SensorType & _tp, const Eigen::VectorXs & _pose, const Eigen::VectorXs & _params) :
    type_(_tp), 
	sensor_pose_vehicle_(_pose), 
	params_(_params.size())
{
    params_ = _params;
}

SensorBase::SensorBase(const SensorType & _tp, const Eigen::VectorXs & _pose, unsigned int _params_size) : 
    type_(_tp), 
    sensor_pose_vehicle_(_pose), 
    params_(_params_size)
{
    //
}

SensorBase::~SensorBase()
{
    //
}

const SensorType SensorBase::getSensorType() const
{
    return type_;
}

const Eigen::VectorXs * SensorBase::getSensorPose() const
{   
    return & sensor_pose_vehicle_;
}

