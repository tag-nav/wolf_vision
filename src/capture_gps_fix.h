#ifndef CAPTURE_GPS_FIX_H_
#define CAPTURE_GPS_FIX_H_

//Wolf includes
#include "capture_base.h"
#include "feature_gps_fix.h"

//std includes
//

//class CaptureGPSFix
class CaptureGPSFix : public CaptureBase
{
    protected:
        Eigen::VectorXs data_; ///< Raw data.
        Eigen::MatrixXs data_covariance_; ///< Noise of the capture.

    public:
        CaptureGPSFix(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data);

        CaptureGPSFix(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~CaptureGPSFix();

        virtual void process();

        virtual Eigen::VectorXs computeFramePose(const TimeStamp& _now) const;

        //virtual void printSelf(unsigned int _ntabs = 0, std::ostream & _ost = std::cout) const;
};
#endif
