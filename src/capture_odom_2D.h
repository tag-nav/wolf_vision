
#ifndef CAPTURE_ODOM_2D_H_
#define CAPTURE_ODOM_2D_H_

//std includes
//

//Wolf includes
#include "capture_relative.h"
#include "feature_odom_2D.h"
#include "sensor_odom_2D.h"

//class CaptureGPSFix
class CaptureOdom2D : public CaptureRelative
{

    public:
      CaptureOdom2D(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr);

      CaptureOdom2D(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr, const Eigen::Vector3s& _data);

      CaptureOdom2D(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr, const Eigen::Vector3s& _data, const Eigen::Matrix3s& _data_covariance);
        
      virtual ~CaptureOdom2D();

      virtual void processCapture();

      virtual Eigen::VectorXs computePrior(const TimeStamp& _now = 0) const;

      virtual void addConstraints();

      virtual void integrateCapture(CaptureRelative* _new_capture);

      virtual CaptureOdom2D* interpolateCapture(const TimeStamp& _ts);

      //virtual void printSelf(unsigned int _ntabs = 0, std::ostream & _ost = std::cout) const;
};
#endif
