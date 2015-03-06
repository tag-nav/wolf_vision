
#ifndef CAPTURE_ODOM_2D_H_
#define CAPTURE_ODOM_2D_H_

//std includes
//

//Wolf includes
#include "capture_relative.h"
#include "feature_odom_2D.h"

//class CaptureGPSFix
class CaptureOdom2D : public CaptureRelative
{
    public:
		CaptureOdom2D(const TimeStamp& _ts, SensorBase* _sensor_ptr);

		CaptureOdom2D(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data);

		CaptureOdom2D(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);
        
        virtual ~CaptureOdom2D();
        
        virtual void processCapture();

        virtual Eigen::VectorXs computePrior() const;

        virtual void addConstraints();

        virtual void integrateCapture(CaptureRelative* _new_capture);

        //virtual void printSelf(unsigned int _ntabs = 0, std::ostream & _ost = std::cout) const;
};
#endif
