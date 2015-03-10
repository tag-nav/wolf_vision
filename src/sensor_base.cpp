#include "sensor_base.h"

SensorBase::SensorBase(const SensorType & _tp, const Eigen::Vector6s & _pose, const Eigen::VectorXs & _params) :
	NodeBase("SENSOR"),
    type_(_tp), 
	sensor_position_vehicle_(_pose.head(3)),
	params_(_params.size())
{
    params_ = _params;
	sensor_rotation_vehicle_ = Eigen::AngleAxisd(_pose(3), Eigen::Vector3d::UnitX()) *
							   Eigen::AngleAxisd(_pose(4), Eigen::Vector3d::UnitY()) *
							   Eigen::AngleAxisd(_pose(5), Eigen::Vector3d::UnitZ());
}

SensorBase::SensorBase(const SensorType & _tp, const Eigen::Vector6s & _pose, unsigned int _params_size) :
	NodeBase("SENSOR"),
    type_(_tp), 
	sensor_position_vehicle_(_pose.head(3)),
    params_(_params_size)
{
	sensor_rotation_vehicle_ = Eigen::AngleAxisd(_pose(3), Eigen::Vector3d::UnitX()) *
							   Eigen::AngleAxisd(_pose(4), Eigen::Vector3d::UnitY()) *
							   Eigen::AngleAxisd(_pose(5), Eigen::Vector3d::UnitZ());
}

SensorBase::~SensorBase()
{
	std::cout << "deleting SensorBase " << nodeId() << std::endl;
}

const SensorType SensorBase::getSensorType() const
{
    return type_;
}

const Eigen::Vector3s * SensorBase::getSensorPosition() const
{
	//std::cout << "getSensorPosition: " << sensor_position_vehicle_.transpose() << std::endl;
    return & sensor_position_vehicle_;
}

const Eigen::Matrix3s * SensorBase::getSensorRotation() const
{   
	//std::cout << "getSensorRotation: " << sensor_rotation_vehicle_ << std::endl;
    return & sensor_rotation_vehicle_;
}


