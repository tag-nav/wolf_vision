#ifndef CAPTURE_FIX_H_
#define CAPTURE_FIX_H_

//Wolf includes
#include "capture_base.h"
#include "feature_fix.h"
#include "constraint_fix.h"

//std includes
//

namespace wolf {


//class CaptureFix
class CaptureFix : public CaptureBase
{
    protected:
        Eigen::VectorXs data_; ///< Raw data.
        Eigen::MatrixXs data_covariance_; ///< Noise of the capture.

    public:

        CaptureFix(const TimeStamp& _ts, SensorBase* _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);

        /** \brief Default destructor (not recommended)
         *
         * Default destructor (please use destruct() instead of delete for guaranteeing the wolf tree integrity)
         *
         **/
        virtual ~CaptureFix();

        virtual void process();

};

} //namespace wolf
#endif
