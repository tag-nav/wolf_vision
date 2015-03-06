#include "sensor_base.h"

SensorBase::SensorBase(const SensorType & _tp, const Eigen::VectorXs & _pose, const Eigen::VectorXs & _params) :
	NodeBase("SENSOR"),
    type_(_tp), 
	sensor_pose_vehicle_(_pose), 
	params_(_params.size())
{
    params_ = _params;
}

SensorBase::SensorBase(const SensorType & _tp, const Eigen::VectorXs & _pose, unsigned int _params_size) : 
	NodeBase("SENSOR"),
    type_(_tp), 
    sensor_pose_vehicle_(_pose), 
    params_(_params_size)
{
    //
}

SensorBase::~SensorBase()
{
	std::cout << "deleting SensorBase " << nodeId() << std::endl;
}

const SensorType SensorBase::getSensorType() const
{
    return type_;
}

const Eigen::VectorXs * SensorBase::getSensorPose() const
{   
    return & sensor_pose_vehicle_;
}

