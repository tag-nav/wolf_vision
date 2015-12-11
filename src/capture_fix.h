#ifndef CAPTURE_FIX_H_
#define CAPTURE_FIX_H_

//std includes
//

//Wolf includes
#include "capture_base.h"
#include "feature_fix.h"
#include "constraint_fix.h"

//class CaptureFix
class CaptureFix : public CaptureBase
{
    public:

        CaptureFix(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~CaptureFix();

        virtual void processCapture();

        virtual Eigen::VectorXs computePrior(const TimeStamp& _now) const;
};
#endif
