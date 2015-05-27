#include "sensor_base.h"

SensorBase::SensorBase(const SensorType & _tp, StatePoint3D* _p_ptr, StateOrientation* _o_ptr, const Eigen::VectorXs & _params) :
        NodeBase("SENSOR"),
        type_(_tp),
        p_ptr_(_p_ptr),
        o_ptr_(_o_ptr),
        params_(_params.size())
{
    params_ = _params;
}

SensorBase::SensorBase(const SensorType & _tp, StatePoint3D* _p_ptr, StateOrientation* _o_ptr, unsigned int _params_size) :
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

StatePoint3D* SensorBase::getPPtr() const
{
    return p_ptr_;
}

StateOrientation* SensorBase::getOPtr() const
{
    return o_ptr_;
}
