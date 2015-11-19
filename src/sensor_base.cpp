#include "sensor_base.h"

SensorBase::SensorBase(const SensorType & _tp, StateBase* _p_ptr, StateBase* _o_ptr, const Eigen::VectorXs & _params) :
        NodeBase("SENSOR"),
        type_(_tp),
        p_ptr_(_p_ptr),
        o_ptr_(_o_ptr),
        params_(_params.size())
{
    params_ = _params;
}

SensorBase::SensorBase(const SensorType & _tp, StateBase* _p_ptr, StateBase* _o_ptr, unsigned int _params_size) :
        NodeBase("SENSOR"),
        type_(_tp),
        p_ptr_(_p_ptr),
        o_ptr_(_o_ptr),
        params_(_params_size)
{
    //
}

SensorBase::~SensorBase()
{
    //std::cout << "deleting SensorBase " << nodeId() << std::endl;
}

const SensorType SensorBase::getSensorType() const
{
    return type_;
}

StateBase* SensorBase::getPPtr() const
{
    return p_ptr_;
}

StateBase* SensorBase::getOPtr() const
{
    return o_ptr_;
}

Eigen::Matrix2s SensorBase::getRotationMatrix2D() {
	// TODO: move this code somewhere else and do a real get()
	assert ( o_ptr_->getStateType() != ST_QUATERNION && "2D rot matrix not defined for quaternions." );
		return Eigen::Rotation2D<WolfScalar>(*(o_ptr_->getPtr())).matrix();
}

Eigen::Matrix3s SensorBase::getRotationMatrix3D() {
	// TODO: move this code somewhere else and do a real get()
    Eigen::Matrix3s R = Eigen::Matrix3s::Identity();

    if ( o_ptr_->getStateType() == ST_QUATERNION )
		R.block<2,2>(0,0) = Eigen::Rotation2D<WolfScalar>(*(o_ptr_->getPtr())).matrix();

    else
		R = Eigen::Map<Eigen::Quaternions>(o_ptr_->getPtr()).toRotationMatrix();

    return R;
}
