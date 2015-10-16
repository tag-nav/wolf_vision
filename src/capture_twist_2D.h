
#ifndef CAPTURE_TWIST_2D_H_
#define CAPTURE_TWIST_2D_H_

//std includes
//

//Wolf includes
#include "capture_relative.h"
#include "feature_twist_2D.h"
#include "sensor_twist_2D.h"

class CaptureTwist2D : public CaptureRelative
{
    public:
      CaptureTwist2D(const TimeStamp& _init_ts, SensorTwist2D* _sensor_ptr);

      CaptureTwist2D(const TimeStamp& _init_ts, SensorTwist2D* _sensor_ptr, const Eigen::Vector3s& _data);

      CaptureTwist2D(const TimeStamp& _init_ts, SensorTwist2D* _sensor_ptr, const Eigen::Vector3s& _data, const Eigen::Matrix3s& _data_covariance);
        
      virtual ~CaptureTwist2D();

      virtual void processCapture();

      virtual Eigen::VectorXs computePrior(const TimeStamp& _now) const;

      virtual void addConstraints();

      virtual void integrateCapture(CaptureRelative* _new_capture);

      virtual CaptureTwist2D* interpolateCapture(const TimeStamp& _ts);

      //virtual void printSelf(unsigned int _ntabs = 0, std::ostream & _ost = std::cout) const;
};
#endif
