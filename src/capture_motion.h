
#ifndef CAPTURE_RELATIVE_H_
#define CAPTURE_RELATIVE_H_

//std includes
//

//Wolf includes
#include "wolf.h"
#include "capture_base.h"

//class CaptureBase
class CaptureMotion : public CaptureBase
{
    protected:
        TimeStamp final_time_stamp_; ///< Final Time stamp

    public:
        CaptureMotion(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr);
        
        CaptureMotion(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data);

        CaptureMotion(const TimeStamp& _init_ts, const TimeStamp& _final_ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~CaptureMotion();

        virtual void integrateCapture(CaptureMotion* _new_capture) = 0;

        virtual CaptureMotion* interpolateCapture(const TimeStamp& _ts) = 0;

        TimeStamp getInitTimeStamp() const;

        TimeStamp getFinalTimeStamp() const;

        void setInitTimeStamp(const TimeStamp & _ts);

        void setFinalTimeStamp(const TimeStamp & _ts);
};
#endif
