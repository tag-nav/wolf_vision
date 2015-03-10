#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_

//std includes
#include <iostream>

//Wolf includes
#include "wolf.h"
#include "node_base.h"

class SensorBase : public NodeBase
{
    protected:
        SensorType type_;//indicates sensor type. Enum defined at wolf.h
        Eigen::Vector3s sensor_position_vehicle_;//sensor position in the vehicle frame
        Eigen::Matrix3s sensor_rotation_vehicle_;//sensor rotation in the vehicle frame
        Eigen::VectorXs params_;//sensor intrinsic params: offsets, scale factors, sizes, ... 
        //bool generate_prior_; //flag indicating if this sensor generates the prior or not
    
    public:
        SensorBase(const SensorType & _tp, const Eigen::Vector6s & _pose, const Eigen::VectorXs & _params);
        
        SensorBase(const SensorType & _tp, const Eigen::Vector6s & _pose, unsigned int _params_size);

        ~SensorBase();
        
        const SensorType getSensorType() const;
        
        const Eigen::Vector3s * getSensorPosition() const;

        const Eigen::Matrix3s * getSensorRotation() const;
        
};
#endif

