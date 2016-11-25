#ifndef CAPTURE_FIX_H_
#define CAPTURE_FIX_H_

//Wolf includes
#include "capture_base.h"
#include "feature_fix.h"
#include "constraint_fix.h"
#include "constraint_fix_3D.h"

//std includes
//

namespace wolf {

WOLF_PTR_TYPEDEFS(CaptureFix);

//class CaptureFix
class CaptureFix : public CaptureBase
{
    protected:
        Eigen::VectorXs data_; ///< Raw data.
        Eigen::MatrixXs data_covariance_; ///< Noise of the capture.

    public:

        CaptureFix(const TimeStamp& _ts, SensorBasePtr _sensor_ptr, const Eigen::VectorXs& _data, const Eigen::MatrixXs& _data_covariance);

        virtual ~CaptureFix();

        virtual void process();

};

} //namespace wolf
#endif
