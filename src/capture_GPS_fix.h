#ifndef CAPTURE_GPS_FIX_H_
#define CAPTURE_GPS_FIX_H_

//Wolf includes
#include <feature_GPS_fix.h>
#include "capture_base.h"

//std includes
//

namespace wolf {
    
WOLF_PTR_TYPEDEFS(CaptureGPSFix);

//class CaptureGPSFix
class CaptureGPSFix : public CaptureBase
{
    protected:
        Eigen::VectorXs data_; ///< Raw data.
        Eigen::MatrixXs data_covariance_; ///< Noise of the capture.

    public:
        CaptureGPSFix(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const Eigen::VectorXs& _data);
        CaptureGPSFix(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);
        virtual ~CaptureGPSFix();
        virtual void process();
};

} //namespace wolf
#endif
