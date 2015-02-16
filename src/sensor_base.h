#ifndef SENSOR_BASE_H_
#define SENSOR_BASE_H_


//std includes
//

//Wolf includes
#include "wolf.h"

class SensorBase
{
    protected:
        SensorType type_;//indicates sensor type. Enum defined at wolf.h
        Eigen::VectorXs sensor_pose_vehicle_;//sensor pose in the vehicle frame
        Eigen::VectorXs params_;//sensor intrinsic params: offsets, scale factors, sizes, ... 
        //bool generate_prior_; //flag indicating if this sensor generates the prior or not
    
    public:
        SensorBase(const SensorType & _tp, const Eigen::VectorXs & _pose, const Eigen::VectorXs & _params);
        
        SensorBase(const SensorType & _tp, const Eigen::VectorXs & _pose, unsigned int _params_size);

        ~SensorBase();
        
        const SensorType getSensorType() const;
        
        const Eigen::VectorXs * getSensorPose() const;
};
#endif

